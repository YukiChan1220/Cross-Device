#include <SPI.h>

#define MASTER_SCK_PIN 12
#define MASTER_MOSI_PIN 13
#define MASTER_MISO_PIN 14
#define SLAVE1_SS_PIN A1
#define SLAVE2_SS_PIN A2
#define SLAVE3_SS_PIN A3
#define SLAVE4_SS_PIN A4
#define SLAVE5_SS_PIN A5

const size_t bufferSize = 64;  // Adjusted for 14 floats (14 * 4 bytes = 56 bytes)
uint8_t receivedBuffer[bufferSize];
float receivedData[14];
bool pollingActive[5] = { false, false, false, false, false };  // Polling status for each slave
unsigned long lastPollTime[5] = { 0, 0, 0, 0, 0 };          // Track last polling time for each slave
unsigned long startTime[5] = { 0, 0, 0, 0, 0 };           // Start time for each slave
unsigned long endTime[5] = { 0, 0, 0, 0, 0 };                // End time for each slave
const unsigned long pollInterval = 9;                    // Poll interval for each slave (milliseconds)
int sampleCounter[5] = { 0, 0, 0, 0, 0 };                   // Sample counters for each slave
int nanCounter[5] = { 0, 0, 0, 0, 0 };                       // NaN counters for each slave

String getSSPinName(int pin) {
  if (pin == A0) return "A0";
  if (pin == A1) return "A1";
  if (pin == A2) return "A2";
  if (pin == A3) return "A3";
  if (pin == A4) return "A4";
  if (pin == A5) return "A5";
  return String(pin);  // Default to numeric value if not an analog pin
}

void setup() {
  Serial.begin(1000000);
  while (!Serial)
    ;

  pinMode(SLAVE1_SS_PIN, OUTPUT);
  pinMode(SLAVE2_SS_PIN, OUTPUT);
  pinMode(SLAVE3_SS_PIN, OUTPUT);
  pinMode(SLAVE4_SS_PIN, OUTPUT);
  pinMode(SLAVE5_SS_PIN, OUTPUT);

  digitalWrite(SLAVE1_SS_PIN, HIGH);
  digitalWrite(SLAVE2_SS_PIN, HIGH);
  digitalWrite(SLAVE3_SS_PIN, HIGH);
  digitalWrite(SLAVE4_SS_PIN, HIGH);
  digitalWrite(SLAVE5_SS_PIN, HIGH);

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  Serial.println("Cross");
}

void sendCommandToSlave(int slaveNumber, const char* command) {
  memset(receivedBuffer, 0, sizeof(receivedBuffer));

  int slavePins[] = { SLAVE1_SS_PIN, SLAVE2_SS_PIN, SLAVE3_SS_PIN, SLAVE4_SS_PIN, SLAVE5_SS_PIN };
  int currentPin = slavePins[slaveNumber - 1];

  // Select the correct slave
  for (int i = 0; i < 5; i++) {
    digitalWrite(slavePins[i], i == (slaveNumber - 1) ? LOW : HIGH);
  }

  // Perform SPI transfer for the command
  for (size_t i = 0; i < bufferSize; i++) {
    receivedBuffer[i] = SPI.transfer((i < strlen(command)) ? command[i] : 0x00);
  }

  for (int i = 0; i < 5; i++) {
    digitalWrite(slavePins[i], HIGH);
  }

  /*Serial.print("Command sent to Slave ");
  Serial.print(slaveNumber);
  Serial.print(" (Pin: ");
  Serial.print(getSSPinName(currentPin));
  Serial.println(").");*/
}

void startAllSlaves() {
  for (int i = 0; i < 5; i++) {
    if (!pollingActive[i]) {
      sendCommandToSlave(i + 1, "s");
      pollingActive[i] = true;
      sampleCounter[i] = 0;     // Reset sample counter
      nanCounter[i] = 0;        // Reset NaN counter
      startTime[i] = millis();  // Record start time
    }
  }
  Serial.println("All slaves started.");
}

void stopSlave(int slaveNumber) {
  int index = slaveNumber - 1;
  if (pollingActive[index]) {
    // Send stop command to the slave
    sendCommandToSlave(slaveNumber, "e");

    // Calculate duration and sampling rate
    endTime[index] = millis();
    unsigned long durationMs = endTime[index] - startTime[index];
    float durationSec = durationMs / 1000.0;                    // Convert to seconds
    float samplingRate = (sampleCounter[index] / durationSec);  // Samples per second
    float realsamplingRate = (sampleCounter[index] - nanCounter[index]) / durationSec;
    // Calculate NaN percentage
    float nanPercentage = (sampleCounter[index] > 0) ? (nanCounter[index] * 100.0 / sampleCounter[index]) : 0;

    // Print results
    Serial.println("===== Slave " + String(slaveNumber) + " Transmission Summary =====");
    Serial.print("Total number: ");
    Serial.println(sampleCounter[index]);
    Serial.print("Total Real number: ");
    Serial.println(sampleCounter[index] - nanCounter[index]);
    Serial.print("Frequency: ");
    Serial.print(samplingRate, 2);
    Serial.println(" Hz");
    Serial.print("Real Frequency (No NaN): ");
    Serial.print(realsamplingRate, 2);
    Serial.println(" Hz");
    Serial.print("Duration: ");
    Serial.print(durationSec, 2);
    Serial.println(" s");
    Serial.print("NaN Count: ");
    Serial.println(nanCounter[index]);
    Serial.print("NaN Percentage: ");
    Serial.print(nanPercentage, 2);  // Format with 2 decimal places
    Serial.println(" %");
    Serial.println("=====================================");

    // Reset polling status
    pollingActive[index] = false;
  }
}


void stopAllSlaves() {
  for (int i = 0; i < 5; i++) {
    if (pollingActive[i]) {
      stopSlave(i + 1);  // Stop each active slave
    }
  }
  Serial.println("All slaves stopped.");
}

void receiveDataFromSlave(int slaveNumber) {
  memset(receivedBuffer, 0, sizeof(receivedBuffer));
  memset(receivedData, 0, sizeof(receivedData));

  int slavePins[] = { SLAVE1_SS_PIN, SLAVE2_SS_PIN, SLAVE3_SS_PIN, SLAVE4_SS_PIN, SLAVE5_SS_PIN };
  int currentPin = slavePins[slaveNumber - 1];

  // Select the correct slave
  for (int i = 0; i < 5; i++) {
    digitalWrite(slavePins[i], i == (slaveNumber - 1) ? LOW : HIGH);
  }

  // Perform SPI transfer to fetch data
  for (size_t i = 0; i < bufferSize; i++) {
    receivedBuffer[i] = SPI.transfer(0x00);
  }

  for (int i = 0; i < 5; i++) {
    digitalWrite(slavePins[i], HIGH);
  }

  float currentTime = (float)millis();
  memcpy(&receivedBuffer[13 * sizeof(float)], &currentTime, sizeof(float));

  memcpy(receivedData, receivedBuffer, sizeof(receivedData));
  checkForSpecificNaN(receivedData, slaveNumber);
  // printreceiveddata(receivedData, 14, slaveNumber, currentPin);

  // Increment the sample counter for the slave
  int index = slaveNumber - 1;
  sampleCounter[index]++;
  Serial.write(receivedBuffer, 64);
}

void checkForSpecificNaN(float data[], int slaveNumber) {
  String numberAsString = String(data[0], 2);  // Convert with 2 decimal places
  // Serial.println(numberAsString);

  if (numberAsString == "-nan") {  // Example: -NaN in IEEE 754
    nanCounter[slaveNumber - 1] += 1;
  }
}

void printreceiveddata(float data[], int length, int slaveNumber, int slavePin) {


  bool nanDetected = false;
  Serial.print("Received data from Slave ");
  Serial.print(slaveNumber);
  Serial.print(" (Pin: ");
  Serial.print(getSSPinName(slavePin));
  Serial.print("): [");



  for (int i = 0; i < length; i++) {


    Serial.print(data[i], 2);  // Print each value with 2 decimal places
    if (i < length - 1) {
      Serial.print(", ");  // Add a comma and space for all but the last element
    }
  }
  Serial.println("]");
  if (nanDetected) {
    Serial.print("WARNING: Detected NaN in data from Slave ");
    Serial.print(slaveNumber);
    Serial.print(" (Pin: ");
    Serial.print(getSSPinName(slavePin));
    Serial.println(").");
  }
}


void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "s") {
      startAllSlaves();  // Start all slaves
    } else if (input == "e") {
      stopAllSlaves();  // Stop all slaves
    } else {
      Serial.println("Invalid input! Use 's' to start all slaves or 'e' to stop all slaves.");
    }
  }

  // Poll each active slave based on its interval
  unsigned long currentTime = millis();
  for (int i = 0; i < 5; i++) {
    if (pollingActive[i] && (currentTime - lastPollTime[i] >= pollInterval)) {
      lastPollTime[i] = currentTime;
      receiveDataFromSlave(i + 1);
    }
  }
}
