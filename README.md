# 🚀 STM32 Nucleo-F446RE – ADXL345, OLED, and Bluetooth Controlled Servo System

## 📘 Project Overview

This project implements a **servo control and tilt measurement system** using the **STM32 Nucleo-F446RE** development board.
The system reads tilt data from an **ADXL345 accelerometer**, calculates roll and pitch angles, displays them on a **0.96" OLED screen**, and allows **servo control via accelerometer**. And it also allows **manual servo control via Bluetooth** using the **HM-10 module**.

---

## 🧩 Hardware Components

| Component              | Function                               | Communication |
| ---------------------- | -------------------------------------- | ------------- |
| STM32 Nucleo-F446RE    | Main microcontroller                   | —             |
| ADXL345 Accelerometer  | Measures tilt (roll/pitch)             | I2C           |
| SSD1306 OLED (0.96")   | Displays roll, pitch, and servo data   | I2C           |
| SG90 Servo Motor       | Moves based on tilt or Bluetooth input | PWM           |
| HM-10 Bluetooth Module | Enables wireless servo control         | UART          |

---

## ⚙️ Hardware Connections

| Component                | STM32 Pin        | Interface |
| ------------------------ | ---------------- | --------- |
| **ADXL345** SDA          | PB9              | I2C1 SDA  |
| **ADXL345** SCL          | PB8              | I2C1 SCL  |
| **OLED (SSD1306)** SDA   | PB9              | I2C1 SDA  |
| **OLED (SSD1306)** SCL   | PB8              | I2C1 SCL  |
| **Servo Motor (PWM)**    | PA6              | TIM3_CH1  |
| **Bluetooth (HM-10)** TX | PA10 (USART1 RX) | UART      |
| **Bluetooth (HM-10)** RX | PA9 (USART1 TX)  | UART      |
| VCC                      | 3.3V / 5V        | —         |
| GND                      | GND              | —         |

---

## 💡 System Operation

1. The **ADXL345** sensor measures acceleration along the X, Y, and Z axes.

2. Roll and pitch angles are calculated from the raw data.

3. The **servo motor** is automatically positioned based on the roll angle or can be manually controlled via Bluetooth.

4. The **OLED display** shows:

   * Roll angle
   * Pitch angle
   * System status (“STABLE” or “TILTED”)
   * Current servo angle

5. **Bluetooth control mode:**

   * Connect to the HM-10 module via a terminal app (e.g., *BLESerial Tiny*).
   * Send a value between `0` and `180` followed by a newline (`\n`).
     Example:

     ```
     90\n
     ```
   * The servo rotates to the given angle, and the OLED updates the displayed value.

---

## 🔢 Servo Control Details

Servo motor control is performed via **PWM** on `TIM3_CH1`.

| Servo Angle | PWM Pulse Width |
| ----------- | --------------- |
| 0°          | 500 µs          |
| 90°         | 1500 µs         |
| 180°        | 2500 µs         |

PWM generation formula:

```c
uint16_t pwm_value = (uint16_t)((angle / 180.0f) * 2000.0f + 500.0f);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_value);
```

---

## 🧠 Software Structure

### 📂 Main File: `main.c`

**Key Modules and Functions**

| Function                    | Description                               |
| --------------------------- | ----------------------------------------- |
| `ADXL345_Init()`            | Initializes the ADXL345 sensor            |
| `ADXL345_ReadXYZ()`         | Reads raw acceleration data               |
| `ADXL345_ComputeAngles()`   | Calculates roll and pitch angles          |
| `Servo_Set_From_Angle()`    | Sets servo position using PWM             |
| `HAL_UART_RxCpltCallback()` | Handles Bluetooth input and updates servo |
| `ssd1306_WriteString()`     | Displays text on OLED screen              |

---

## 📊 OLED Display Example

```
Roll : -12.35
Pitch:  4.72
Status: STABLE
Servo:  45.00
```

If the board is tilted more than ±15°:

```
Status: TILTED
```

---

## 🔌 UART (Bluetooth) Configuration

| Parameter | Value                    |
| --------- | ------------------------ |
| Baud Rate | 9600                     |
| Data Bits | 8                        |
| Stop Bits | 1                        |
| Parity    | None                     |
| Mode      | RX & TX (Interrupt mode) |

UART communication is handled using `HAL_UART_Receive_IT()` interrupt callbacks.

---

## 🖥️ How to Use

1. Connect the STM32 board and open the project in **STM32CubeIDE**.
2. Build and flash the code to the Nucleo board.
3. Power the system and connect your phone or PC to the **HM-10 Bluetooth module**.
4. Open a terminal app and send servo angle values (0–180).
5. Observe:

   * Servo movement according to tilt or Bluetooth command.
   * Real-time roll/pitch angles and servo position displayed on OLED.

---

## 📸 Project Summary

This project integrates **sensor data processing, UART communication, PWM control, and OLED visualization** into a single embedded system.
It demonstrates multiple fundamental embedded communication protocols:

* **I2C** → ADXL345 & OLED
* **UART** → Bluetooth communication
* **PWM (TIM3)** → Servo control

---

## 🧾 Technical Specifications

| Specification      | Value                       |
| ------------------ | --------------------------- |
| MCU                | STM32F446RE (ARM Cortex-M4) |
| Clock Frequency    | 180 MHz                     |
| PWM Timer          | TIM3                        |
| UART Port          | USART1                      |
| I2C Speed          | 100 kHz                     |
| Servo Range        | 0° – 180°                   |
| Bluetooth Protocol | UART/Serial (HM-10 BLE)     |

---

## 🧰 Libraries Used

* **STM32 HAL Drivers** (via STM32CubeIDE)
* **ssd1306.h / ssd1306_fonts.h** ([SSD1306 OLED display library](https://github.com/afiskon/stm32-ssd1306/tree/master).)
* **math.h**, **stdio.h**, **stdlib.h** (Standard C libraries)

---

## 📜 License

This project is based on STM32 HAL libraries provided by STMicroelectronics.
Free for educational and non-commercial use.

---
