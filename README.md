---

# ⚗️ Automated Acid Diluting System

[![Project License](https://img.shields.io/badge/license-MIT-green)](LICENSE) ![Project Status](https://img.shields.io/badge/status-Active-brightgreen)

Welcome to the **Automated Acid Diluting System** repository! This project aims to improve safety and efficiency in laboratories by automating the process of diluting acids using a microcontroller-based system. By reducing human errors and minimizing exposure to hazardous chemicals, this system brings cutting-edge technology to laboratory processes.

---

## 📋 Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [System Components](#system-components)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [Contributors](#contributors)

---

## 🔬 Introduction

Mixing acids is a critical and potentially dangerous task. Our project introduces an automated system that handles acid dilution, ensuring accurate mixtures while significantly reducing human error. This system is ideal for laboratories, especially those in educational institutions and research facilities, where safety is paramount.

### 🛡️ Why This System?
Manual acid dilution poses several risks:
- Incorrect acid concentrations
- Accidental spills
- Harmful chemical exposure
- Lack of expertise in educational environments

Our **microcontroller-based system** solves these issues by automating the process, ensuring safety, precision, and reliability.

---

## ✨ Features

- **Automated Acid Dilution**: Reduces manual handling and human errors.
- **Real-time Safety Checks**: Built-in safety features, including emergency shutdown and environmental monitoring.
- **Precise Measurement**: Utilizes Time-of-Flight (TOF) sensors to ensure accurate water measurements for dilution.
- **Multi-sensor Integration**: Uses gyroscope, ultrasonic, and temperature sensors for enhanced control.
- **User-Friendly**: Simple keypad interface and LCD display for ease of operation.
- **Cost Efficient**: Designed with affordable and widely available components.

---

## 🛠️ System Components

- **ESP32 Microcontroller (2x)**: For communication and control.
- **VL53L0X Time-of-Flight Sensor**: Measures water level precisely.
- **Ultrasonic Sensors (2x)**: Detects the presence of the output beaker and checks for liquid.
- **DS18B20 Temperature Sensor**: Monitors the temperature of the liquid.
- **GY-521 MPU-6050 3-Axis Gyroscope**: Ensures the system is level.
- **Water Pump**: Controls the flow of water.
- **5V 2-Position Water Solenoid Valve**: Manages water output.
- **DC Motors**: Operates various mechanical components of the system.
- **L298N DC Motor Driver**: Controls the speed and direction of the motors.
- **2-Channel 5V Relay Module**: Controls the water pump and solenoid valve.
- **Buck Converters**: Steps down voltage to power various system components.

---

## 🖥️ Installation

### Prerequisites
- Arduino IDE for programming
- ESP32 Boards (with ESP-NOW communication protocol enabled)
- Required hardware components (listed above)

### Setup Steps
1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/automated-acid-dilution.git
   cd automated-acid-dilution
   ```
2. **Install Dependencies**:
   Ensure all libraries for the ESP32, sensors, and motor drivers are installed.
   
   In the Arduino IDE, install the following libraries:
   - ESP32
   - esp_now
   - WiFi
   - Wire
   - LiquidCrystal_I2C
   - Keypad
   - DallasTemperature.h
   - Adafruit_VL53L0X

---

This section provides an easy-to-understand overview of the critical libraries and interfaces used in your system, emphasizing the importance of each in the overall operation. It’s placed in a separate section for easy reference, which enhances clarity and readability.

3. **Upload Code to ESP32 Boards**:
   - Connect the ESP32 boards to your system and upload the respective code.
   
4. **Connect Hardware**:
   - Follow the schematic to connect the sensors, motors, and power supplies correctly.

---

## 🚀 Usage

1. Power on the system.
2. Input the required parameters through the keypad.
3. The system will automatically dilute the acid to the desired concentration, with real-time feedback on the LCD display.
4. Safety mechanisms like emergency shutoff will engage if any abnormality is detected.

---

## 📁 Project Structure

```plaintext
├── code/
│   ├── main.ino            # Arduino code for the main system
│   ├── esp32_communication.ino  # ESP-NOW communication setup
├── hardware/
│   ├── schematic.png        # Schematic of the system
│   ├── block_diagram.png    # Block diagram for component connections
├── documentation/
│   ├── user_manual.pdf      # Detailed user manual
│   ├── presentation.pptx    # Project presentation
├── LICENSE                  # License file
└── README.md                # This README file
```

---

## 👨‍💻 Contributors

- **R.E.M.H.M. Rajakaruna**: [ToF Sensor Integration, Mechanical Design]
- **G.D. Punchihewa**: [ESP32 Communication, Water Pump Control]
- **H.M.M.D. Herath**: [Gyroscope, Buck Converters, PCB Design]
- **S.J. Mayadunna**: [Ultrasonic Sensors, LCD Display, Keypad]
- **C.B.R.N.D. Bandara**: [Temperature Sensor, Buzzer, Finishing]

Feel free to contribute to the project by submitting pull requests or reporting issues!

---

## 🎉 Acknowledgments

- Special thanks to [University of Moratuwa](https://uom.lk) Faculty of Information Technology for supporting this project.
- Inspiration from existing automated chemical systems in large-scale industrial factories.

---
