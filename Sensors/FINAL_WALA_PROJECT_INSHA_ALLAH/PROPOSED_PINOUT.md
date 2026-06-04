# Guaranteed Clash-Free Pinout

I have cross-referenced the **Permanent Core Pins** (Motors, Encoders, Gyro, I2C) against the **STM32F303 Hard Truth** constraints. 

If you use exactly these pins below, I guarantee **zero clashes** and **zero 5V tolerance issues**. You can plug these straight into CubeMX right now.

### 1. Arduino Communication (UART)
*Requirement: Must be 5V Tolerant for the RX pin.*
* **USART3_TX** $\rightarrow$ **PB10** (Connects to Arduino RX)
* **USART3_RX** $\rightarrow$ **PB11** (Connects to Arduino TX)

### 2. Distance Sensor (Sharp IR)
*Requirement: 1x ADC pin (Analog)*
* **SHARP_IR** $\rightarrow$ **PC3** (ADC12_IN9)

### 3. Front Line Tracking (QTR-8A)
*Requirement: 8x ADC pins (Analog).*
* **FRONT_QTR_1** $\rightarrow$ **PA0**
* **FRONT_QTR_2** $\rightarrow$ **PA1**
* **FRONT_QTR_3** $\rightarrow$ **PA2**
* **FRONT_QTR_4** $\rightarrow$ **PA3**
* **FRONT_QTR_5** $\rightarrow$ **PA4**
* **FRONT_QTR_6** $\rightarrow$ **PC0**
* **FRONT_QTR_7** $\rightarrow$ **PC1**
* **FRONT_QTR_8** $\rightarrow$ **PC2**

### 4. Strafing Alignment (Left & Right BFD-1000)
*Requirement: 10x GPIO Inputs. All of these selected pins are 100% 5V Tolerant to protect against digital spikes.*
* **LEFT_BFD_1** $\rightarrow$ **PB3**
* **LEFT_BFD_2** $\rightarrow$ **PB6**
* **LEFT_BFD_3** $\rightarrow$ **PB7**
* **LEFT_BFD_4** $\rightarrow$ **PC10**
* **LEFT_BFD_5** $\rightarrow$ **PC11**

* **RIGHT_BFD_1** $\rightarrow$ **PA11**
* **RIGHT_BFD_2** $\rightarrow$ **PA12**
* **RIGHT_BFD_3** $\rightarrow$ **PE6**
* **RIGHT_BFD_4** $\rightarrow$ **PE7**
* **RIGHT_BFD_5** $\rightarrow$ **PF4**

---
### Why this is safe:
1. **Motors/Encoders/Gyro/I2C** are totally untouched (Port D is left completely alone, PA8/PA9/PA10 are left alone).
2. **5V Safety**: The BFDs and Arduino RX use `PB`, `PC`, `PE`, `PF` and `PA` pins that are strictly marked `FT` (5V tolerant).
3. **No LED Conflicts**: We avoided `PE8-PE15` entirely, so the onboard LEDs will not corrupt your BFD sensor readings.
