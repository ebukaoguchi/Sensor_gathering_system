# Sensor_gathering_system

Sensor Data Gathering and Cloud-based Transmitter Prototype System

This project focuses on developing a prototype system that can automatically detect sensor types and transmit the correct value and unit to the cloud. The system was designed to ensure seamless integration with various sensor types, particularly for applications in agricultural monitoring. The main objective is to provide a reliable, cost-effective, and modular solution that researchers can use to gather environmental data remotely.
Features

    Automatic Sensor Type Detection: The system can automatically identify the type of sensor connected (e.g., temperature, soil moisture) based on unique identification voltages.
    Cloud Integration: Sensor data is transmitted and stored on the Microsoft Azure Cloud, where it can be accessed and analyzed remotely.
    Error Logging and Watchdog Timer: Built-in error logging and a watchdog timer ensure the system operates reliably, with safeguards against data loss or incorrect sensor readings.

Skills and Technologies

    Arduino Programming: Implemented the sensor interface and transmission logic using C/C++ on the Arduino platform.
    Circuitry Design and Soldering: Designed and assembled the necessary circuitry for sensor detection and communication.
    Temperature and Soil Moisture Sensors: Integrated with different types of sensors to monitor environmental conditions.
    Microsoft Azure Cloud: Connected the system to the Azure Cloud for data storage and remote access.
    Antenna Design: Designed and tested antennas for long-range wireless communication.
    Schematic and PCB Design: Developed schematics and layouts for the hardware components.

Installation and Setup

To set up this system:

    Connect the batteries to the Transmission Device and the SparkFun ESP32 gateway.
    Attach the required sensor (Temperature or Soil Moisture) to the transmitter.
    Power on the system and register the device in the Microsoft Azure Cloud to begin data transmission.



## Hardware Requirements

- ESP32 or similar microcontroller
- LoRa modules
- Temperature and Soil Moisture sensors
- Antennas
- Power supply (batteries)
- Supporting components (resistors, connectors, etc.)

## Software Requirements

- Arduino IDE
- Required Libraries:
  - `LoRa`
  - `WiFi`
  - `Adafruit_SSD1306` (for OLED display)
  - `ArduinoJson` (for handling JSON data)

## Setup Instructions

1. **Hardware Setup:**
   - Connect the sensors, LoRa module, and other components as per the circuit diagram.

2. **Software Setup:**
   - Install the Arduino IDE.
   - Install the required libraries using the Library Manager in the Arduino IDE.
   - Load the `.ino` files into the Arduino IDE.

3. **Compiling and Uploading:**
   - Select your board (e.g., ESP32) and the appropriate COM port.
   - Compile the code to check for errors.
   - Upload the code to the microcontroller.

## Code Files

- **_otaServer.ino**: Handles Over-The-Air (OTA) updates.
- **oLED.h** and **_oLED.ino**: Manage the OLED display for visual feedback.
- **loraModem.h** and **_loraModem.ino**: Control the LoRa communication module.
- **loraFiles.h** and **_loraFiles.ino**: File management for LoRa transmissions.
- **_gatewayMgt.ino**: Manages gateway operations.
- **ESP-sc-gway_Group_1.ino** and **ESP-sc-gway.h**: Main gateway sketch and header.
- **_wwwServer.ino**: Handles the web server for device management.
- **_WiFi.ino**: Manages Wi-Fi connections.
- **_utils.ino**: Utility functions used across the project.
- **_txRx.ino**: Transmission and reception logic.
- **_stateMachine.ino**: Manages the state of the system.
- **sensor.h** and **_sensor.ino**: Sensor management and data collection.
- **_repeater.ino**: LoRa repeater functionality.
- **transmitter_code.ino**: Main code for the transmitter.

## Running the Project

1. **Upload the code to the ESP32.**
2. **Power on the device and connect to the Wi-Fi network.**
3. **The system will start collecting sensor data and transmitting it to the cloud.**

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

Special thanks to the team members and instructors who contributed to this project.



Contributors

    Ebuka Philip Oguchi
    Junxiao Zhang
    Md Didarul Islam
