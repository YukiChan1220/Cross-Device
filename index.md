# Cross-Device Configuration Manual
## 1 Environment Configuration
### 1.1 Device Connection
* **Install USB Driver:**  
  The device use CP2104 for USB-UART bridge.  
  Download and install **CP210x Universal Windows Driver** or **CP210x VCP Mac OSX Driver** [here](https://www.silabs.com/developer-tools/usb-to-uart-bridge-vcp-drivers?tab=downloads).  
  **Reboot the computer.**  

* **Check Connection:**
  - **Windows**  
    Use **Micro USB** cable to connect the device to the computer.  
    Open **Device Manager**, extend **Ports (COM & LPT)**, look for **Silicon Labs CP210x USB to UART Bridge (COMn)**, in which **COMn** is the COM port.  
  - **MacOS**  
    **Disconnect** the device with the computer.  
    Open **Terminal**, run `ls /dev/tty.*`.  
    Use **Micro USB** cable to commect the device to the computer.  
    run `ls /dev/tty.*` again, look for new devices like **/dev/tty.SLAB_USBtoUART**, which is the COM port.  
  
### 1.2 Arduino IDE Configuration
Install and open the Arduino IDE.
* **Board Management:**  
  Open **File**->**Preferences**, find **Additional boards manager URLs**, add `https://adafruit.github.io/arduino-board-index/package_adafruit_index.json`.  
  Open **BOARDS MANAGER** in the left slide, install the newest version of **Adafruit nRF52** by Adaruit.

* **Library Management:**  
  Open **LIBRARY MANAGER** in the left slide, install the newest versions of **SparkFun MAX3010x Pulse and Proximity Sensor Library**, **Adafruit TMP117**, **SparkFun 9DoF IMU Breakout - ICM 20948 - Arduino Library**.

### 1.3 Program Upload
* Open the program in Arduino IDE.
* Connect the device to the computer.
* Select **Tools**->**Board**->**Adafruit nRF52**->**Adafruit Feather nRF52832**.
* Select the correct COM port in **Tools**->**Port**.
* Click the **Upload** icon (looks like **->**).
  
## 2 Multiple Slaves Configuration
When using multiple slaves, both the master and slave firmware programs need to be modified in the Arduino IDE and re-uploaded to their respective devices.
### 2.1 Configure Master Device
To configure the number of slaves connected to the master device, make the following changes in **sensor_master**:
* **Macros:**
  Define SS pins for each slave.
  ```cpp
  #define SLAVE1_SS_PIN A1
  #define SLAVE2_SS_PIN A2
  #define SLAVE3_SS_PIN A3
  #define SLAVE4_SS_PIN A4
  #define SLAVE5_SS_PIN A5
  ```
* **Global Variables:**
  Define global variables. Change 5 to the actual numbers of the slaves used.
  ```cpp
  bool pollingActive[5] = { false, false, false, false, false };  // Polling status for each slave
  unsigned long lastPollTime[5] = { 0, 0, 0, 0, 0 };          // Track last polling time for each slave
  unsigned long startTime[5] = { 0, 0, 0, 0, 0 };           // Start time for each slave
  unsigned long endTime[5] = { 0, 0, 0, 0, 0 };                // End time for each slave
  const unsigned long pollInterval = 9;                    // Poll interval for each slave (milliseconds)
  int sampleCounter[5] = { 0, 0, 0, 0, 0 };                   // Sample counters for each slave
  int nanCounter[5] = { 0, 0, 0, 0, 0 };                       // NaN counters for each slave
  ```
* **getSSPinName() function:**
  Modify this function to match the SS pins assigned to each slave.
  ```cpp
  String getSSPinName(int pin) {
    if (pin == A1) return "A1";
    if (pin == A2) return "A2";
    if (pin == A3) return "A3";
    if (pin == A4) return "A4";
    if (pin == A5) return "A5";
    return String(pin);
  }
  ```
* **Pin Configurations:**
  Set the pinMode and initial state for each slave's SS pin.
  ```cpp
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
  ```

* **Loop Configurations:**
 Search for for `(int i = 0; i < 5; i++)` in the code, and change 5 to the actual number of slaves used.

### 2.2 Configure Slave Device
Make the following changes in **sensor_sax**:
* **Macros:**
  Ensure that the SS pin matches the one defined in the master code.
  ```cpp
  #define SPIS_SS_PIN A1
  ```

* **SPI Buffer Settings:**
  Search for the following line and change 0xA1 to 0xA**x**, where **x** represents the index of the slave.
  ```cpp
  memset(m_tx_buf, 0xA1, sizeof(m_tx_buf));
  ```
  **Note: DO NOT** modify `memset(m_tx_buf, 0, sizeof(m_tx_buf));`. It is used to clear the buffer.
  

## 3 PPG Sensor
### 3.1 LED current
Search for `setPulseAmplitude` in Arduino code.  
Adjust the parameters of these functions:
```cpp
particleSensor.setPulseAmplitudeRed(0x80);
particleSensor.setPulseAmplitudeIR(0x80);
particleSensor.setPulseAmplitudeGreen(0xD0);
```
For detailed information, refer to Page 20, Table 8 in the [MAX30101 datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX30101.pdf).

### 3.2 LED Brightness
Search for `ledBrightness` in Arduino code.  
Adjust the value of ledBrightness:
```cpp
byte ledBrightness = 0xFF;  //Options: 0=Off to 255=50mA
```

## 4 Data Collection
### 4.1 Hardware Connection
Ensure the hardware connection matches with the firmware program.  
Connect the master device to the computer.  
Change `PORT` in `receiver.py` to the actual port.
### 4.2 Data Collection and Process
Run `receiver.py`, press **Ctrl+C** to stop data collection.  
Run `dataprocess.py`, it will process raw data and generate csvs in **./processed_data**.

## 5 Debug
Before debugging, always go with the **Checklist** according to the type of the problem. Always try **resetting and reconnecting** first.  
### 5.1 Hardware Checklist
* **USB Cable:** Connected to the master device. Try reconnecting.  
* **FPC Cable:** Connected firmly; Not folded; Not broken; Not reversed.  
* **PCB Boards:** No lighted LEDs. Try resetting.  
* **FPC Boards:** PPG LEDs lighted.  
* **Temperature:** No overheating (60^C or higher) anywhere.  

### 5.2 Software Checklist
* **File Location:** All software under the same folder.  

### 5.3 Hardware debugging
* **PPG LED not lighted; No output data from certain slave:**  
  - Disconnect the USB cable from the master device.  
  - Connect the USB cable to the faulty slave device.  
  - Use Arduino IDE or any port monitoring software to check serial output. **Baudrate 115200**. Reset the slave if no serial output.  
  - Look for any **ERROR** messages.  
  - **If ERROR:** Replace the sensor FPC board and FPC cable with new ones.  
  - **If NO OUTPUT:** Re-upload the slave firmware. Replace FPC board & cable.  
* **No output data from the entire system:**
  - Check the FPC cable between master and slave.  
  - Connect the USB cable to the master device.  
  - Use Arduino IDE or any port monitoring software to check serial output. **Baudrate 1000000**. Reset the master if no serial output.  
  - Check if there's any output.  
  - **If OUTPUT:** Input `s` and press enter; Wait for 3 sec; Then input `e` and press enter; Check the output for NaN count.  
  - **If NO OUTPUT:** Re-upload the master firmware. Replace FPC cable.  

## Report Problems
To report any problems, [open an issue](https://github.com/YukiChan1220/Cross-Device/issues) or contact **SparcYuki** via Wechat.
