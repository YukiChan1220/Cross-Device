#include <nrfx.h>
#include <nrfx_spis.h>
#include <Arduino.h>  // For random number generation and millis()


#include <Wire.h>
#include <MAX30105.h>         // PPG sensor library
#include <Adafruit_TMP117.h>  // TMP117 temperature sensor library


#define SPIS_SCK_PIN 12   // P0.12 (SCK)
#define SPIS_MOSI_PIN 13  // P0.13 (MOSI)
#define SPIS_MISO_PIN 14  // P0.14 (MISO)
#define SPIS_SS_PIN A1    // P0.03 (CS/SS)

// TWI0 Pins (Default)
#define TWI_SDA_PIN 26  // P0.26 (TWI0 SDA)
#define TWI_SCL_PIN 27  // P0.27 (TWI0 SCL)

// Sensor Objects
MAX30105 particleSensor;
Adafruit_TMP117 tmp117;

// Global Variables
float temperature = 0.0;



// #define SPIS_SCK_PIN 15   // P0.15 (SPI1 SCK)
// #define SPIS_MOSI_PIN 16  // P0.16 (SPI1 MOSI)
// #define SPIS_MISO_PIN 14  // P0.17 (SPI1 MISO)
// #define SPIS_SS_PIN A1    // P0.18 (SPI1 SS)

#define SPIS_INSTANCE 1
static const nrfx_spis_t spis = NRFX_SPIS_INSTANCE(SPIS_INSTANCE);

static const uint8_t m_length = 64;  // Adjusted for 14 floats (14 * 4 bytes = 56 bytes)
static uint8_t m_rx_buf[m_length];   // Buffer for data received from Master
static uint8_t m_tx_buf[m_length];   // Buffer for data to be sent to Master

static volatile bool spis_xfer_done = false;  // Flag for completed SPI transactions
static bool send_sensor_data = false;         // Flag to control sensor data transmission

int counter = 0;                   // Counter for sent data packets
unsigned long start_time = 0;      // Start time of the transaction
unsigned long last_send_time = 0;  // Last time data was sent
unsigned long send_interval = 2;   // Sending interval in milliseconds (default: 10 ms)


float ALL_MAX[14] = {  // Define the ALL_MAX array
  -1, -1, -1,
  -1, -1, -1,
  -1, -1, -1,
  -1, -1, -1,
  -1, -1
};


// TMP117 Initialization
void initializeTMP117() {
  if (!tmp117.begin()) {
    Serial.println("ERROR: Failed to find TMP117 chip.");
    while (1)
      ;  // Halt
  }
  Serial.println("TMP117 Found!");

  // Configure TMP117 (optional settings)
  tmp117.setAveragedSampleCount(TMP117_AVERAGE_1X);  // Use single-sample averaging
  tmp117.setReadDelay(TMP117_DELAY_0_MS);            // No delay between readings
}

// Initialize Sensors
void initializeSensors() {
  // Initialize TWI0 (I2C) for PPG and TMP117 sensors
  Wire.begin();  // Uses default TWI0 pins (P0.26 for SDA, P0.27 for SCL)

  //Setup to sense up to 18 inches, max LED brightness
  byte ledBrightness = 0xFF;  //Options: 0=Off to 255=50mA
  byte sampleAverage = 1;     //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 3;           //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  int sampleRate = 1600;      //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;       //Options: 69, 118, 215, 411
  int adcRange = 16384;       //Options: 2048, 4096, 8192, 16384

  // Initialize MAX30105 PPG sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST, 0x57)) {
    Serial.println("ERROR: MAX30105 not found.");
    while (1)
      ;  // Halt
  }

  // Configure PPG sensor
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);  //Configure sensor. Use 6.4mA for LED drive
  particleSensor.setPulseAmplitudeRed(0x50);
  particleSensor.setPulseAmplitudeIR(0x50);
  particleSensor.setPulseAmplitudeGreen(0xD0);
  Serial.println("MAX30105 Initialized.");

  // Initialize TMP117 sensor
  initializeTMP117();
}

void check_available_one_max() {
  particleSensor.check();
  while (particleSensor.available())  //do we have new data?
  {


    ALL_MAX[0] = particleSensor.getFIFORed();
    ALL_MAX[1] = particleSensor.getFIFOIR();
    ALL_MAX[2] = particleSensor.getFIFOGreen();


    particleSensor.nextSample();
  }
}

// Read Data from Sensors
void readSensorData() {
  // PPG Data
  check_available_one_max();

  // TMP117 Data
  sensors_event_t temp;
  tmp117.getEvent(&temp);  // Retrieve temperature event
  temperature = temp.temperature;

  // Debug output
  // Serial.print("[PPG] Red: ");
  // Serial.print(red);
  // Serial.print(", IR: ");
  // Serial.print(ir);
  // Serial.print(", Green: ");
  // Serial.print(green);
  // Serial.print(", Temp: ");
  // Serial.println(temperature);

  // Prepare SPI buffer
  ALL_MAX[3] = temperature;
}


void printArrayAsList(float data[], int length, int slaveNumber) {
  Serial.print("Received data from Slave ");
  Serial.print(slaveNumber);
  Serial.print(": [");
  for (int i = 0; i < length; i++) {
    Serial.print(data[i], 2);  // Print each value with 2 decimal places
    if (i < length - 1) {
      Serial.print(", ");  // Add a comma and space for all but the last element
    }
  }
  Serial.println("]");  // Close the array format
}

// Function to generate random dummy data
void generateDummyData() {
  for (int i = 0; i < 14; i++) {
    ALL_MAX[i] = random(-100, 101) / 10.0;  // Generate random floats between -10.0 and 10.0
  }
  printArrayAsList(ALL_MAX, 14, 2);
}

// Function to map SS pin to its symbolic name
String getSSPinName(int pin) {
  if (pin == A0) return "A0";
  if (pin == A1) return "A1";
  if (pin == A2) return "A2";
  if (pin == A3) return "A3";
  return String(pin);  // Default to numeric value if not an analog pin
}

// SPI event handler
void spis_event_handler(nrfx_spis_evt_t const* event, void* context) {
  if (event->evt_type == NRFX_SPIS_XFER_DONE) {
    spis_xfer_done = true;  // Indicate that the SPI transaction is done
  }
}

void setup() {
  Serial.begin(115200);  // Start serial communication for debugging

  initializeSensors();

  nrfx_spis_config_t spis_config = NRFX_SPIS_DEFAULT_CONFIG(SPIS_SCK_PIN, SPIS_MOSI_PIN, SPIS_MISO_PIN, SPIS_SS_PIN);
  nrfx_err_t result = nrfx_spis_init(&spis, &spis_config, spis_event_handler, nullptr);
  if (result != NRFX_SUCCESS) {
    Serial.print("SPIS initialization failed. Error code: ");
    Serial.println(result);
    while (true)
      ;
  }

  memset(m_rx_buf, 0, sizeof(m_rx_buf));
  memset(m_tx_buf, 0xA1, sizeof(m_tx_buf));

  result = nrfx_spis_buffers_set(&spis, m_tx_buf, sizeof(m_tx_buf), m_rx_buf, sizeof(m_rx_buf));
  if (result != NRFX_SUCCESS) {
    Serial.print("Failed to set SPI buffers. Error code: ");
    Serial.println(result);
    while (true)
      ;
  }

  Serial.println("SPIS initialized and ready.");

  randomSeed(analogRead(0));  // Initialize random seed
}

void loop() {
  if (spis_xfer_done) {
    spis_xfer_done = false;

    String command = String((char*)m_rx_buf);
    Serial.print("Command: ");
    Serial.println(command);
    command.trim();

    // Handle "start" command
    if (command == "s") {
      send_sensor_data = true;
      memset(m_tx_buf, 0xA1, sizeof(m_tx_buf));
      counter = 0;                // Reset the counter
      start_time = millis();      // Record the start time
      last_send_time = millis();  // Reset last send time
      Serial.println("Start sending sensor data.");
    }
    // Handle "stop" command
    else if (command == "e") {
      send_sensor_data = false;
      memset(m_tx_buf, 0, sizeof(m_tx_buf));  // Clear the TX buffer

      // Calculate transmission details
      unsigned long end_time = millis();
      float duration = (end_time - start_time) / 1000.0;  // Duration in seconds
      float frequency = counter / duration;               // Frequency in Hz

      // Print the final summary
      Serial.println("===== Transmission Summary =====");
      Serial.print("SS PIN Address: ");
      Serial.println(getSSPinName(SPIS_SS_PIN));
      Serial.print("Total number: ");
      Serial.println(counter);
      Serial.print("Frequency: ");
      Serial.print(frequency, 2);  // Format with 2 decimal places
      Serial.println(" Hz");
      Serial.print("Duration: ");
      Serial.print(duration, 2);  // Format with 2 decimal places
      Serial.println(" s");
      Serial.println("================================");
    }
    // Handle "set interval" command (e.g., "i100" to set interval to 100 ms)
    else if (command.startsWith("i")) {
      String intervalString = command.substring(1);  // Extract interval value
      send_interval = intervalString.toInt();
      Serial.print("Send interval set to: ");
      Serial.print(send_interval);
      Serial.println(" ms");
    }

    memset(m_rx_buf, 0, sizeof(m_rx_buf));  // Clear RX buffer


    nrfx_err_t result = nrfx_spis_buffers_set(&spis, m_tx_buf, sizeof(m_tx_buf), m_rx_buf, sizeof(m_rx_buf));
    if (result != NRFX_SUCCESS) {
      Serial.print("Failed to reset SPI buffers. Error code: ");
      Serial.println(result);
    }
  }

  if (send_sensor_data) {
    unsigned long current_time = millis();
    if (current_time - last_send_time >= send_interval) {
      last_send_time = current_time;  // Update the last send time
      counter++;                      // Increment counter for each data packet

      // Generate random data
      // generateDummyData();

      // Get sensor data
      int tmp_stime = micros();
      readSensorData();
      int tmp_etime = micros();
      Serial.print("time: ");
      Serial.println(tmp_etime - tmp_stime);
      
      // Copy random data to SPI transmit buffer
      memcpy(m_tx_buf, ALL_MAX, sizeof(ALL_MAX));




      // Debug print for counter
      // Serial.print("Counter: ");
      // Serial.println(counter);
    }
  }
}
