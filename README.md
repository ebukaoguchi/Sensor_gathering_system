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

Contributors

    Ebuka Philip Oguchi
    Junxiao Zhang
    Md Didarul Islam
