# Environmental-Sensing-Project

## Overview

This project implements firmware for an environmental sensing device that reads data from multiple sensors and transmits aggregated statistics periodically. The system is built on an STM32F4 series microcontroller and uses I2C to interface with sensors for temperature, humidity, and pressure. Although the design originally intended to use BLE for data transmission, UART has been used in this implementation due to hardware constraints.

## Flowchart
<p align="center">
  <img src="https://github.com/user-attachments/assets/037a5a08-6b4d-49bf-baa0-2f4daf4bd89f" alt="Description" />
</p>




---

## Project Objectives

- **Sensor Data Acquisition:** Sample environmental parameters (temperature, humidity, and pressure) at 1 Hz using three I2C sensors.
- **Data Filtering:** Implement a moving median filter to smooth out sensor data.
- **Data Buffering:** Store filtered values in a circular buffer.
- **Data Transmission:** Transmit aggregated sensor statistics every 30 seconds using UART.

---

## System Architecture

| Component           | Description                                         |
|---------------------|-----------------------------------------------------|
| MCU                 | STM32F446RE (ARM Cortex-M4)                         |
| IDE                 | STM32CubeIDE                                        |
| Interface           | I2C (sensors), UART (transmission)                  |
| Interrupt Timers    | TIM3 (1 Hz for sampling), TIM2 (30s for sending)    |

| Sensor    | Type        | Address | Notes                                             | Datasheets                                                         |
|-----------|-------------|---------|---------------------------------------------------|--------------------------------------------------------------------|
| LM75A     | Temperature | 0x48    | 9-bit resolution, ±2°C accuracy                   | https://www.ti.com/lit/ds/symlink/lm75a.pdf                        |
| Si7021    | Humidity    | 0x40    | ±3% RH accuracy, fixed I2C address                | https://www.silabs.com/documents/public/data-sheets/Si7021-A20.pdf |
| LPS25HB   | Pressure    | 0x5C    | Digital sensor with WHO_AM_I and CTRL registers   | https://www.st.com/resource/en/datasheet/lps25hb.pdf               |

---

## ⏱️ Timer Configuration

Two hardware timers manage system tasks:

- **Timer 3 (TIM3):**  
  - Configured to trigger an interrupt every 1 second for sensor data acquisition.
  - Example configuration snippet:
    ```c
    // 1 Hz configuration: (84MHz / (Prescaler+1) / (Period+1)) = 1 Hz
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 8399;
    htim3.Init.Period = 9999;
    HAL_TIM_Base_Init(&htim3);
    ```
![int1sec](https://github.com/user-attachments/assets/3bad1a19-124b-476a-ab4f-4d0b80dc61bb)
    
- **Timer 2 (TIM2):**  
  - Configured to trigger an interrupt every 30 seconds to transmit data.
  - Example configuration snippet:
    ```c
    // 30-second configuration: (84MHz / (Prescaler+1) / (Period+1)) = 1/30 Hz
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 8399;
    htim2.Init.Period = 29999;
    HAL_TIM_Base_Init(&htim2);
    ```
![int30sec](https://github.com/user-attachments/assets/b62c0a2c-012a-4936-8490-efcbfc174fdb)

    

These timers are verified using a logic analyzer, confirming the precise 1 Hz and 30-second intervals.

---

## Detailed Implementation

### Sensor Interfaces

This section explains, step by step, how sensor readings are performed via the I2C interface using the implemented code.

### LM75A – Temperature Sensor

- **Initialization (`LM75A_Init`):**
  - **Purpose:** Verify sensor presence before starting temperature measurements.
  - **Steps:**
    - **I2C Memory Read:**  
      - Reads 1 byte from register `0x00` using `HAL_I2C_Mem_Read` with address `LM75A_ADDR` (shifted left by 1).
    - **Error Handling:**  
      - If the read operation fails, return `HAL_ERROR`; otherwise, return `HAL_OK`.

- **Temperature Reading (`LM75A_ReadTemperature`):**
  - **Steps:**
    - **Data Acquisition:**  
      - Reads 2 bytes from register `0x00` using `HAL_I2C_Mem_Read`.
    - **Data Combination:**  
      - Combines the two bytes into a 16-bit raw temperature value:
        ```c
        int16_t raw_temp = (data[0] << 8) | data[1];
        ```
    - **Extracting 9-bit Value:**  
      - Shifts the raw value right by 7 bits to obtain a 9-bit two's complement number:
        ```c
        int16_t temp_9bit = raw_temp >> 7;
        ```
    - **Handling Negative Values:**  
      - Checks if the sign bit (bit 8) is set. If yes, computes the two's complement to convert the value to a negative temperature.
    - **Conversion:**  
      - Multiplies the result by 0.5 to convert to °C and returns the value.

---

### Si7021 – Humidity & Temperature Sensor

- **Initialization (`Si7021_Init`):**
  - **Purpose:** Confirm sensor identity.
  - **Steps:**
    - **Transmit Command:**  
      - Sends command `0xFA` to initiate reading the electronic ID.
    - **Receive Data:**  
      - Receives 2 bytes of the electronic ID.
    - **Validation:**  
      - Checks if the first byte equals `0x06` (expected value). Returns `HAL_ERROR` if not matching; else returns `HAL_OK`.

- **Humidity Reading (`Si7021_ReadHumidity`):**
  - **Steps:**
    - **Command Initiation:**  
      - Sends command `0xE5` to measure relative humidity.
    - **Delay:**  
      - Uses `HAL_Delay(20)` to wait for conversion completion.
    - **Data Acquisition:**  
      - Receives 2 bytes of humidity data.
    - **Data Combination:**  
      - Combines the bytes into a 16-bit raw value.
    - **Conversion:**  
      - Converts raw data to RH using:
        ```c
        humidity = ((125.0f * raw_humidity) / 65536.0f) - 6.0f;
        ```
      - Returns the computed humidity.

---

### LPS25HB – Pressure Sensor

- **Initialization (`LPS25HB_Init`):**
  - **Purpose:** Verify sensor identity and configure output data rate.
  - **Steps:**
    - **Identity Check:**  
      - Reads the `WHO_AM_I` register (`0x0F`) using `HAL_I2C_Mem_Read`.
      - Compares the returned value with `0xBD`; returns `HAL_ERROR` if mismatched.
    - **Configuration:**  
      - Writes to `CTRL_REG1` (`0x20`) with value `0b10000000` to power the sensor on and set the output data rate to 1 Hz.
      - Returns `HAL_OK` upon success.

- **Pressure Reading (`LPS25HB_ReadPressure`):**
  - **Steps:**
    - **Data Acquisition:**  
      - Reads 3 bytes from register `0x28` using `HAL_I2C_Mem_Read`.
    - **Data Combination:**  
      - Combines the 3 bytes into a 24-bit raw pressure value:
        ```c
        int32_t raw_pressure = (data[2] << 16) | (data[1] << 8) | data[0];
        ```
    - **Conversion:**  
      - Divides the raw value by `4096.0f` to convert it to hPa.
      - Returns the computed pressure.
---

### Data Filtering and Buffering

- **Moving Median Filter:**  
  - A sliding window is maintained for each sensor.  
  - Bubble sort is applied on the window data to compute the median.

- **Circular Buffer:**  
  - Filtered sensor data is stored in a fixed-size circular buffer.  
  - When the buffer is full, the oldest data is overwritten.

### Data Read & Transmission

- **Sampling:**  
  - Timer interrupts trigger sensor sampling every second.
  - Sensor data is either acquired from hardware or simulated.
  
- **Transmission:**  
  - Every 30 seconds, sensor data is collected.
  - Statistics (standard deviation, max, min, and median) are calculated and transmitted over UART.

---

## Simulation & Testing

- **Mock Data:**  
- Since physical sensors were not available during development, sensor values are simulated using randomized functions.
- Real sensor code is included in `main.c` as commented sections for easy activation when hardware is available.

- **Verification:**  
- **UART Transmission:** Captured via a serial terminal, showcasing the data packet format.
![pcktOut](https://github.com/user-attachments/assets/9bef8cad-ab91-4bd6-a9f8-380d912d269d)

- A short video demonstrating UART transmission:

https://github.com/user-attachments/assets/33b4392e-658a-4985-b325-56ed46d2b83c




