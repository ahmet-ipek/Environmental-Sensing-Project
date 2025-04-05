# Environmental-Sensing-Project

## Overview

This project implements firmware for an environmental sensing device that reads data from multiple sensors and transmits aggregated statistics periodically. The system is built on an STM32F4 series microcontroller and uses I2C to interface with sensors for temperature, humidity, and pressure. Although the design originally intended to use BLE for data transmission, UART has been used in this implementation due to hardware constraints.

---

## Table of Contents

- [Project Objectives](#project-objectives)
- [System Architecture](#system-architecture)
- [Detailed Implementation](#detailed-implementation)
  - [Sensor Interfaces](#sensor-interfaces)
  - [Data Filtering and Buffering](#data-filtering-and-buffering)
  - [Data Transmission](#data-transmission)
- [Development Tools and Datasheets](#development-tools-and-datasheets)
- [Deviations and Future Work](#deviations-and-future-work)
- [Getting Started](#getting-started)
- [License](#license)

---

## Project Objectives

- **Sensor Data Acquisition:** Sample environmental parameters (temperature, humidity, and pressure) at 1 Hz using three I2C sensors.
- **Data Filtering:** Implement a moving median filter to smooth out sensor data.
- **Data Buffering:** Store filtered values in a circular buffer.
- **Data Transmission:** Transmit aggregated sensor statistics every 30 seconds using UART (substitute for BLE).

---

## System Architecture

- **MCU:** STM32F446RE microcontroller.
- **Sensors:**
  - **LM75A:** Temperature sensor (I2C address: 0x48).
  - **Si7021:** Humidity sensor (I2C address: 0x40).
  - **LPS25HB:** Pressure sensor (I2C address: 0x5C).
- **Communication:**
  - **I2C:** Used to interface with sensors.
  - **UART:** Used for transmitting sensor data and debug information.
- **Interrupts:**
  - **Timer 3:** Triggers sensor sampling at 1 Hz.
  - **Timer 2:** Triggers data transmission every 30 seconds.

---

## Detailed Implementation

### Sensor Interfaces

- **LM75A Temperature Sensor:**  
  - I2C Interface : Supports up to 8 devices on the same bus via address pins (A2, A1, A0).
  - 9-bit Temperature Resolution : Accuracy of ±2°C (–25°C to 100°C) and ±3°C (–55°C to 125°C).
  - Registers :
  - Temperature Register (0x00) : Read-only, updates every 100ms (or 300ms at extreme temps).

- **Si7021 Humidity Sensor:**
  The Si7021-A20 is a digital humidity and temperature sensor with:
  - I2C Interface : 7-bit address 0x40 (fixed, no address pins).
  - Humidity : ±3% RH (0–80% RH).
  - Registers :
  - Humidity/Temperature Measurement Commands (0xE5, 0xE3).
  - Electronic ID Register for device identification.

- **LPS25HB Pressure Sensor:**
  The LPS25HB is a digital pressure sensor with:
  - I2C Interface : Default I2C address 0x5C (SA0=0).
  - Key Registers :
  - WHO_AM_I (0x0F) : Identification register (default 0xBD).
  - CTRL_REG1 (0x20) : Controls power and ODR.

### Data Filtering and Buffering

- **Moving Median Filter:**  
  - A sliding window is maintained for each sensor.  
  - Bubble sort is applied on the window data to compute the median.

- **Circular Buffer:**  
  - Filtered sensor data is stored in a fixed-size circular buffer.  
  - When the buffer is full, the oldest data is overwritten.

### Data Transmission

- **Sampling:**  
  - Timer interrupts trigger sensor sampling every second.
  - Sensor data is either acquired from hardware or simulated.
  
- **Transmission:**  
  - Every 30 seconds, sensor data is aggregated.
  - Statistics (standard deviation, max, min, and median) are calculated and transmitted over UART.
  - Real BLE communication code is prepared but commented out due to hardware unavailability.

---

## Development Tools and Datasheets

### Development Tools

- **MCU:** STM32F446RE microcontroller.
- **IDE:** STM32CubeIDE.
- **Libraries:** STM32 HAL libraries for peripheral management (I2C, UART, Timers).

### Sensor Datasheets

1. **LM75A Temperature Sensor:**
   - **Features:** 9-bit digital temperature sensor, low power.
   - **Range:** –55°C to +125°C.
   - [LM75A Datasheet](https://www.ti.com/lit/ds/symlink/lm75a.pdf)

2. **Si7021 Humidity Sensor:**
   - **Features:** High accuracy humidity and temperature sensor.
   - **Range:** Humidity 0–100% RH; Temperature –10°C to +85°C.
   - [Si7021 Datasheet](https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf)

3. **LPS25HB Pressure Sensor:**
   - **Features:** MEMS-based absolute pressure sensor.
   - **Range:** Approximately 260–1260 hPa.
   - [LPS25HB Datasheet](https://www.st.com/resource/en/datasheet/lps25hb.pdf)

