# STM32F303 Pin Mapping Reference (Full Robot Rearchitecture)

This document serves as a master reference for all physical pin connections on your STM32. It is divided into two sections: **Permanent Core Pins** (which we are keeping untouched) and **New Sensor Suite Pins** (which you will be assigning in CubeMX during the clean slate setup).

---

## 1. Permanent Core Pins (Do Not Change)
These pins are already correctly configured for the robot's essential movement, odometry, and internal sensing. They will remain intact when we create the clean slate project.

### Motors & Drivers
| Component | Function | Port/Pin |
| :--- | :--- | :--- |
| **Front Left Motor** | PWM | PD6 |
| | DIR 1 | PD8 |
| | DIR 2 | PD9 |
| **Front Right Motor**| PWM | PD3 |
| | DIR 1 | PD2 |
| | DIR 2 | PD1 |
| **Rear Left Motor** | PWM | PD7 |
| | DIR 1 | PB5 |
| | DIR 2 | PB4 |
| **Rear Right Motor** | PWM | PD4 |
| | DIR 1 | PD0 |
| | DIR 2 | PC12 |
| **Motor Driver** | Standby (STBY)| PD5 |

### Encoders (Hardware Timers)
| Component | Function | Port/Pin |
| :--- | :--- | :--- |
| **Front Left Encoder**| Channel A | PC6 |
| | Channel B | PD15 |
| **Front Right Encoder**| Channel A | PA9 |
| | Channel B | PA10 |
| **Rear Left Encoder** | Channel A | PC8 |
| | Channel B | PC7 |
| **Rear Right Encoder**| Channel A | PA8 |
| | Channel B | PC9 |

### Gyroscope & Internal (SPI / System)
| Component | Function | Port/Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Gyro (I3G4250D)** | SPI1 SCK | PA5 | Hardware SPI |
| | SPI1 MISO | PA6 | Hardware SPI |
| | SPI1 MOSI | PA7 | Hardware SPI |
| | Chip Select | PD12 | Custom GPIO Output (`GYRO_CS`) |
| **System** | SWDIO | PA13 | Debugging |
| | SWCLK | PA14 | Debugging |

### Color Sensor (I2C)
| Component | Function | Port/Pin |
| :--- | :--- | :--- |
| **TCS34725** | I2C1 SCL | PB8 |
| | I2C1 SDA | PB9 |

---

## 2. New Sensor Suite (To Be Assigned in CubeMX)
These are the pins you will need to manually configure in your new `FINAL_WALA_PROJECT_INSHA_ALLAH.ioc` file. 

You need to find available pins for the following **21** signals:

### A. Front Line Tracking (QTR-8A)
**Task:** Analog line tracking for the main forward movement.
**Requirement:** **8x ADC Pins** (Analog Input)
**Suggested Setup:** Look for pins labeled `ADC1_INx`, `ADC2_INx`, etc. You can group them however you want (e.g., PA0-PA4, PC0-PC3).
- `FRONT_QTR_1`
- `FRONT_QTR_2`
- `FRONT_QTR_3`
- `FRONT_QTR_4`
- `FRONT_QTR_5`
- `FRONT_QTR_6`
- `FRONT_QTR_7`
- `FRONT_QTR_8`

### B. Strafing Alignment (BFD-1000)
**Task:** Reading digital line crossings to stay perfectly parallel while strafing sideways.
**Requirement:** **10x GPIO Pins** (Input mode, preferably with Pull-Ups enabled)
**Suggested Setup:** Any available GPIO pin.
- `LEFT_BFD_1` to `LEFT_BFD_5`
- `RIGHT_BFD_1` to `RIGHT_BFD_5`

### C. Distance Maintenance (Sharp IR)
**Task:** Maintaining an exact distance from the shelf before scanning/shooting.
**Requirement:** **1x ADC Pin** (Analog Input)
- `SHARP_IR_FRONT`

### D. Arduino Communication
**Task:** Sending high-level string commands (`<SHOOT>`, `<SKIP>`) to the Arduino coprocessor.
**Requirement:** **2x UART Pins** (Hardware UART TX/RX)
**Suggested Setup:** Enable `USART2` or `USART3` in Asynchronous mode in CubeMX. It will automatically assign a TX and RX pin. 
- `ARDUINO_TX` (Connects to Arduino RX)
- `ARDUINO_RX` (Connects to Arduino TX)

---

## Wiring Checklist for Phase 1
When you sit down with CubeMX for Phase 1, have this document open. Once you lock in the 21 new pins, save the `.ioc`, generate the code, and we can immediately begin writing the software drivers for them!
