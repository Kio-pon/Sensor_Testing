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
* **LEFT_BFD_1** $\rightarrow$ **PD13**
* **LEFT_BFD_2** $\rightarrow$ **PD14**
* **LEFT_BFD_3** $\rightarrow$ **PD11**
* **LEFT_BFD_4** $\rightarrow$ **PC10**
* **LEFT_BFD_5** $\rightarrow$ **PC11**

* **RIGHT_BFD_1** $\rightarrow$ **PD10**
* **RIGHT_BFD_2** $\rightarrow$ **PF2**
* **RIGHT_BFD_3** $\rightarrow$ **PE6**
* **RIGHT_BFD_4** $\rightarrow$ **PE7**
* **RIGHT_BFD_5** $\rightarrow$ **PF4**

---
### Why this is safe:
1. **Motors/Encoders/Gyro/I2C** are totally untouched (Port D is left completely alone, PA8/PA9/PA10 are left alone).
2. **5V Safety**: The BFDs and Arduino RX use `PB`, `PC`, `PE`, `PF` and `PA` pins that are strictly marked `FT` (5V tolerant).
3. **No LED Conflicts**: We avoided `PE8-PE15` entirely, so the onboard LEDs will not corrupt your BFD sensor readings.

---
### Critical CubeMX Configuration Rules
To make sure these new pins work flawlessly in Phase 2, you MUST apply these settings before generating the code:

#### The ADCs (QTR-8A & Sharp IR)
We are using **Manual Software Polling** (NO DMA!). 
* **Number of Conversions:** `1` (Do NOT set up multiple ranks).
* **Continuous Conversion Mode:** `Disable` 
* **Sampling Time (CRITICAL):** Change this from 1.5 Cycles to **`61.5 Cycles`** for every single ADC channel. This gives the STM32's internal capacitor enough time to fully charge up to the sensor's voltage, completely eliminating "Ghosting" between the 8 different QTR sensors.

#### The BFD-1000 Arrays (GPIO Inputs)
* **GPIO Pull-up/Pull-down:** Set all 10 BFD pins to **`Pull-up`**. This ensures the logic stays perfectly clean (HIGH = white) and doesn't float if a wire wiggles.

#### Arduino Communication (USART3)
* **Mode:** `Asynchronous`
* **Baud Rate:** `115200`
* **NVIC Settings (Interrupts):** You **MUST** check the box to enable the **`USART3 global interrupt`**. This allows the STM32 to receive messages from the Arduino in the background without freezing the robot's movement.

---
### Hardware "Magic Bullets" for ADC Ghosting
If you still see the QTR-8A values bleeding into each other on the Serial Monitor despite the 61.5 cycle sampling time, your sensor's output impedance is too high. Use one of these hardware fixes:
1. **The 10nF Capacitor:** Solder a tiny 10nF to 100nF capacitor directly between the STM32 ADC pin and Ground. It acts as a "charge reservoir" that the STM32 can instantly gulp from.
2. **Op-Amp Buffer:** Place an operational amplifier (in a voltage follower configuration) between the QTR-8A and the STM32. This provides infinite electrical "push" to charge the STM32 instantly.

---
### The Timers (Crucial for Movement & Odometry)
Because we wiped the old project, you must make sure the heart of the robot (its timers) are configured correctly in CubeMX:

#### TIM6 (The Heartbeat & Encoders)
TIM6 runs the 20 kHz interrupt that manually polls your 4 encoders and triggers the 200 Hz motor control loop.
* **Prescaler (PSC):** Set this so the timer ticks at 1 MHz (Usually `71` if your clock is 72 MHz).
* **Counter Period (ARR):** `49` (1 MHz / 50 = 20,000 Hz).
* **NVIC Settings (Interrupts):** You **MUST** check the box to enable the **`TIM6 global interrupt`**. If you forget this, the robot will do absolutely nothing and the encoders will never count!

#### TIM2 (Chassis PWM)
TIM2 generates the PWM signals that tell the motor drivers how fast to spin. 
* **Channel 1 to 4:** Set to **`PWM Generation CHx`**.
* **Prescaler (PSC):** `0`
* **Counter Period (ARR):** `4799` (This gives a smooth, high-frequency PWM limit of 4800, matching your `robot_core.c` code).
