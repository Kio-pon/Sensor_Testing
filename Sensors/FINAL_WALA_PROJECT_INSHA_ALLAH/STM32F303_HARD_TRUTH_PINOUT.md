# STM32F303VCT6 "Hard Truth" Pin Capabilities

When building a complex robot, you cannot guess which pins do what. The STM32F303VCT6 (used on the STM32F3 Discovery) has strict rules about which pins can read analog (ADC), which can do communication (UART/I2C), and which will fry the chip if you give them 5V. 

This is your master reference pulled from the **official ST Datasheet (DS9118)** and **User Manual (UM1570)**.

> [!CAUTION]
> **The 5V Tolerance Rule:** 
> If a pin is marked **FT** (Five-Volt Tolerant), you can safely connect 5V logic signals to it (like a 5V Arduino TX line or a 5V digital sensor). 
> If a pin is **NOT** 5V tolerant, feeding it anything higher than 3.3V will physically damage the microcontroller!

---

## 1. Port A (PA0 to PA15)
*Port A is heavily packed with ADCs and primary communication lines.*

| Pin | 5V Tolerant? | ADC Channels | Best Alternate Functions (AF) |
| :--- | :---: | :--- | :--- |
| **PA0** | ❌ No | ADC1_IN1 | `TIM2_CH1`, `USART2_CTS` |
| **PA1** | ❌ No | ADC1_IN2 | `TIM2_CH2`, `USART2_RTS` |
| **PA2** | ❌ No | ADC1_IN3 | `TIM2_CH3`, **`USART2_TX`** |
| **PA3** | ❌ No | ADC1_IN4 | `TIM2_CH4`, **`USART2_RX`** |
| **PA4** | ❌ No | ADC2_IN1 | `SPI1_NSS`, `DAC1_OUT1` |
| **PA5** | ❌ No | ADC2_IN2 | **`SPI1_SCK`** (Used for Gyro) |
| **PA6** | ❌ No | ADC2_IN3 | **`SPI1_MISO`** (Used for Gyro) |
| **PA7** | ❌ No | ADC2_IN4 | **`SPI1_MOSI`** (Used for Gyro) |
| **PA8** | ✅ **YES** | - | `TIM1_CH1`, `I2C2_SMBA` |
| **PA9** | ✅ **YES** | - | **`USART1_TX`**, `TIM1_CH2` |
| **PA10**| ✅ **YES** | - | **`USART1_RX`**, `TIM1_CH3` |
| **PA11**| ✅ **YES** | - | `USB_DM`, `CAN_RX` |
| **PA12**| ✅ **YES** | - | `USB_DP`, `CAN_TX` |
| **PA13**| ✅ **YES** | - | **`SWDIO`** (DO NOT USE - Debugger) |
| **PA14**| ✅ **YES** | - | **`SWCLK`** (DO NOT USE - Debugger) |
| **PA15**| ✅ **YES** | - | `SPI1_NSS`, `SPI3_NSS`, `I2C1_SCL` |

---

## 2. Port B (PB0 to PB15)
*Port B mixes ADCs, I2C, and Timers.*

| Pin | 5V Tolerant? | ADC Channels | Best Alternate Functions (AF) |
| :--- | :---: | :--- | :--- |
| **PB0** | ❌ No | ADC3_IN12 | `TIM3_CH3` |
| **PB1** | ❌ No | ADC3_IN1 | `TIM3_CH4` |
| **PB2** | ❌ No | ADC2_IN12 | - |
| **PB3** | ✅ **YES** | - | `SPI1_SCK`, `SPI3_SCK`, `USART2_TX` |
| **PB4** | ✅ **YES** | - | `SPI1_MISO`, `SPI3_MISO`, `USART2_RX` |
| **PB5** | ✅ **YES** | - | `I2C1_SMBA`, `I2C3_SDA` |
| **PB6** | ✅ **YES** | - | **`I2C1_SCL`**, **`USART1_TX`** |
| **PB7** | ✅ **YES** | - | **`I2C1_SDA`**, **`USART1_RX`** |
| **PB8** | ✅ **YES** | - | **`I2C1_SCL`**, `CAN_RX` |
| **PB9** | ✅ **YES** | - | **`I2C1_SDA`**, `CAN_TX` |
| **PB10**| ✅ **YES** | - | **`USART3_TX`**, `I2C2_SCL` |
| **PB11**| ✅ **YES** | - | **`USART3_RX`**, `I2C2_SDA` |
| **PB12**| ❌ No | ADC4_IN3 | `I2C2_SMBA`, `USART3_CK` |
| **PB13**| ❌ No | ADC3_IN5 | `USART3_CTS` |
| **PB14**| ❌ No | ADC4_IN4 | `USART3_RTS` |
| **PB15**| ❌ No | ADC4_IN5 | `RTC_REFIN` |

---

## 3. Port C (PC0 to PC15)
*Port C is your analog powerhouse. The first 6 pins are exclusively dedicated to ADCs.*

| Pin | 5V Tolerant? | ADC Channels | Best Alternate Functions (AF) |
| :--- | :---: | :--- | :--- |
| **PC0** | ❌ No | ADC1_IN6 / ADC2_IN6 | - |
| **PC1** | ❌ No | ADC1_IN7 / ADC2_IN7 | - |
| **PC2** | ❌ No | ADC1_IN8 / ADC2_IN8 | - |
| **PC3** | ❌ No | ADC1_IN9 / ADC2_IN9 | - |
| **PC4** | ❌ No | ADC2_IN5 | **`USART1_TX`** |
| **PC5** | ❌ No | ADC2_IN11| **`USART1_RX`** |
| **PC6** | ✅ **YES** | - | `TIM8_CH1`, `TIM3_CH1` |
| **PC7** | ✅ **YES** | - | `TIM8_CH2`, `TIM3_CH2` |
| **PC8** | ✅ **YES** | - | `TIM8_CH3`, `TIM3_CH3` |
| **PC9** | ✅ **YES** | - | `TIM8_CH4`, `TIM3_CH4` |
| **PC10**| ✅ **YES** | - | **`USART3_TX`**, `UART4_TX` |
| **PC11**| ✅ **YES** | - | **`USART3_RX`**, `UART4_RX` |
| **PC12**| ✅ **YES** | - | **`UART5_TX`** |
| **PC13**| ❌ No | - | `RTC_OUT` (Avoid using for heavy loads) |
| **PC14**| ❌ No | - | `OSC32_IN` (Crystal Oscillator) |
| **PC15**| ❌ No | - | `OSC32_OUT` (Crystal Oscillator) |

---

## 4. Port D (PD0 to PD15)
*Port D is purely digital. Almost every pin is 5V tolerant, making it perfect for motor drivers, encoders, and GPIO.*

| Pin | 5V Tolerant? | ADC Channels | Best Alternate Functions (AF) |
| :--- | :---: | :--- | :--- |
| **PD0** | ✅ **YES** | - | `CAN_RX` |
| **PD1** | ✅ **YES** | - | `CAN_TX` |
| **PD2** | ✅ **YES** | - | `UART5_RX` |
| **PD3** | ✅ **YES** | - | `USART2_CTS` |
| **PD4** | ✅ **YES** | - | `USART2_RTS` |
| **PD5** | ✅ **YES** | - | **`USART2_TX`** |
| **PD6** | ✅ **YES** | - | **`USART2_RX`** |
| **PD7** | ✅ **YES** | - | `USART2_CK` |
| **PD8** | ✅ **YES** | - | **`USART3_TX`** |
| **PD9** | ✅ **YES** | - | **`USART3_RX`** |
| **PD10**| ✅ **YES** | - | `USART3_CK` |
| **PD11**| ✅ **YES** | - | `USART3_CTS` |
| **PD12**| ✅ **YES** | - | `USART3_RTS`, `TIM4_CH1` |
| **PD13**| ✅ **YES** | - | `TIM4_CH2` |
| **PD14**| ✅ **YES** | - | `TIM4_CH3` |
| **PD15**| ✅ **YES** | - | `TIM4_CH4` |

---

## 5. Port E (PE0 to PE15)
*Port E is entirely 5V tolerant, but on the STM32F3 Discovery board, many of these pins are hardwired to onboard components!*

| Pin | 5V Tolerant? | ADC Channels | Best Alternate Functions & Discovery Board Conflicts |
| :--- | :---: | :--- | :--- |
| **PE0** | ✅ **YES** | - | `TIM4_ETR` / **MEMS INT1** (Hardwired) |
| **PE1** | ✅ **YES** | - | `TIM17_CH1` / **MEMS INT2** (Hardwired) |
| **PE2** | ✅ **YES** | - | `TIM3_ETR` / **MEMS DRDY** (Hardwired) |
| **PE3** | ✅ **YES** | - | `TIM3_CH1` / **MEMS CS_I2C/SPI** (Hardwired) |
| **PE4** | ✅ **YES** | - | `TIM3_CH2` / **MEMS INT3** (Hardwired) |
| **PE5** | ✅ **YES** | - | `TIM3_CH3` / **MEMS INT4** (Hardwired) |
| **PE6** | ✅ **YES** | - | `RTC_TAMP3` (Free to use) |
| **PE7** | ✅ **YES** | - | `TIM1_ETR` (Free to use) |
| **PE8** | ✅ **YES** | - | `TIM1_CH1N` / **LD4 (Blue LED)** |
| **PE9** | ✅ **YES** | - | `TIM1_CH1` / **LD3 (Red LED)** |
| **PE10**| ✅ **YES** | - | `TIM1_CH2N` / **LD5 (Orange LED)** |
| **PE11**| ✅ **YES** | - | `TIM1_CH2` / **LD7 (Green LED)** |
| **PE12**| ✅ **YES** | - | `TIM1_CH3N` / **LD9 (Blue LED)** |
| **PE13**| ✅ **YES** | - | `TIM1_CH3` / **LD10 (Red LED)** |
| **PE14**| ✅ **YES** | - | `TIM1_CH4` / **LD8 (Orange LED)** |
| **PE15**| ✅ **YES** | - | `TIM1_BKIN` / **LD6 (Green LED)** |

> [!WARNING]
> If you use PE8 through PE15 for sensor inputs (like BFD-1000s), the onboard LEDs will act as pull-down loads and corrupt your signals. Avoid using these for sensors unless you physically desolder the LEDs from the board!

---

## 6. Port F (PF0 to PF10)
*Port F pins are sparse on the 100-pin package. They are generally 5V tolerant but some are used for system clocks.*

| Pin | 5V Tolerant? | ADC Channels | Best Alternate Functions (AF) |
| :--- | :---: | :--- | :--- |
| **PF0** | ✅ **YES** | - | `I2C2_SDA` / **OSC_IN** (System Clock - Do Not Use) |
| **PF1** | ✅ **YES** | - | `I2C2_SCL` / **OSC_OUT** (System Clock - Do Not Use) |
| **PF2** | ✅ **YES** | - | `I2C2_SMBA` (Free to use) |
| **PF4** | ✅ **YES** | - | `TIM3_CH3` (Free to use) |
| **PF6** | ✅ **YES** | - | `TIM4_CH4`, `I2C2_SCL` (Free to use) |
| **PF9** | ✅ **YES** | - | `TIM15_CH1`, `SPI2_SCK` (Free to use) |
| **PF10**| ✅ **YES** | - | `TIM15_CH2`, `SPI2_MISO` (Free to use) |

---

## Hard Rules for Your Rearchitecture

1. **For QTR-8A and Sharp IR (Analog/ADC):**
   - You **MUST** use pins from `PA0-PA7`, `PB0-PB2`, `PB12-PB15`, or `PC0-PC5`. 
   - These pins are **NOT** 5V tolerant, but the analog sensors output 0-3.3V, so it is perfectly safe.
2. **For BFD-1000 Arrays (Digital Input):**
   - You can use ANY available pin, but it is **highly recommended** to use 5V tolerant pins (`FT`), just in case the BFD sensors send a strong logic HIGH. Port D (`PD0-PD15`) is fantastic for this.
3. **For Arduino Communication (UART):**
   - If the Arduino operates at 5V, you **MUST** choose UART pins that are 5V tolerant (`FT`) to receive data.
   - **Good Options:** `PA9/PA10` (USART1), `PB10/PB11` (USART3), or `PD5/PD6` (USART2).
4. **For Motors & Encoders:**
   - Continue using Port D and the top of Port C/Port A just as they were. They are all `FT` (5V tolerant) which makes them bulletproof for motor noise.
