# NERC Line-Following Robot Optimization Guide

**To any future AI Assistant reading this:** This codebase (`FINAL_IOC` / `Robot_Integration`) is for a high-speed line-following robot competing in the NERC (National Engineering Robotics Contest). The hardware runs on an **STM32F303 Cortex-M4 (48 MHz)** with heavy sensor/actuator requirements:
- 23 ADC channels (QTR array + 3 Sharp distance sensors)
- 1 I2C color sensor
- 2 PWM channels for steering/servos
- 4 PWM channels for motor control
- IMU (MPU6050) for tilt/pitch slope detection

To achieve absolute maximum physical responsiveness and zero loop latency, we have carefully tuned the software and build system. **DO NOT introduce blocking code, float division, standard polling, or revert the compiler flags.**

---

## 1. Build System Optimizations (`--fast`)
The custom build wrappers (`flash_windows.sh` and `flash_linux.sh`) run CMake/Ninja with the `--fast` profile. This forces:
- **`-O3`**: Full optimization for speed, enabling aggressive loop unrolling and function inlining.
- **`-flto` (Link Time Optimization)**: Critically important. It analyzes the entire program at link time, removing unused STM32 HAL library code (which is massive). This shrinks the final binary by ~40% (e.g., Color Sensor goes from 30.6KB to 20.4KB), leaving more Flash space and reducing instruction cache pressure.
- **`-ffast-math`**: Configures the Cortex-M4 FPU to use single-cycle fused instructions, maximizing speed for floating-point PID math.

---

## 2. Core Code Rules (No-Jitter & Low Latency)

### Rule A: Never use `HAL_Delay()`
At 1.5 m/s, the robot travels 1.5 mm every single millisecond. A single blocking `HAL_Delay(10)` blinds the robot for 15 mm, causing extreme overshoot and oscillating "swerving."
- **Action**: Always use asynchronous state machines driven by `HAL_GetTick()`.

### Rule B: Avoid Float Division
Floating-point division on the Cortex-M4 FPU takes ~14 clock cycles, whereas multiplication takes only 1 cycle.
- **Action**: Never divide by a variable in hot control loops. If you must divide, pre-calculate the inverse `(1.0f / value)` and multiply by it instead.

### Rule C: Circular DMA for All Sensors
Polling 23 ADC channels and an I2C sensor sequentially will choke the CPU for over 1.2 ms per loop.
- **Action**: Configure ADC and I2C peripherals in **DMA Circular mode**. The CPU should only ever read instantly from the RAM arrays updated by the DMA hardware in the background.

---

## 3. Advanced Embedded C & Control Optimizations (Robotics Secrets)

### Optimization 1: Fixed-Time Timer ISR PID (Zero-Division PID)
Running PID in `while(1)` with a dynamic `dt` (time delta) causes timing jitter due to non-deterministic background tasks. Jitter in `dt` amplifies the Derivative term ($D = de/dt$), creating jerky motor response.
- **Action**: Run the PID inside a Timer Interrupt Service Routine (ISR) (e.g., TIM3 at exactly 1 kHz). Because $dt$ is constant (e.g., $0.001\text{s}$), we pre-integrate $dt$ into the coefficients at startup:
  $$\text{Ki\_prime} = \text{Ki} \times dt$$
  $$\text{Kd\_prime} = \frac{\text{Kd}}{dt}$$
  At runtime, the PID equation reduces to:
  ```c
  integral += error;                              // No float multiply
  derivative = error - previous_error;           // No float division
  output = (kp * error) + (ki_prime * integral) + (kd_prime * derivative);
  ```
  This is 100% immune to jitter and executes in under 20 clock cycles.

### Optimization 2: CCMRAM (Core Coupled Memory RAM) Execution
The STM32F303 has 8KB of CCMRAM connected directly to the Cortex-M4 Instruction/Data buses. Normal SRAM has wait-states due to DMA sharing, but CCMRAM has **zero wait-states**.
- **Action**: Force critical functions (Timer ISR, PID update) and states into CCMRAM to guarantee ultra-fast, deterministic execution:
  ```c
  __attribute__((section(".ccmram")))
  float PID_Update_Fast(PID_t *pid, float error) {
      // Fast PID execution directly from CCMRAM
  }
  ```

### Optimization 3: DMA Double-Buffering (Ping-Pong)
If the CPU reads the ADC DMA array while the DMA is actively transferring data, you get "skewed" readings (half the sensors are from sample $t$, half from sample $t-1$).
- **Action**: Configure a circular DMA buffer of size $2 \times N$.
  - Read from the first half of the buffer inside `HAL_ADC_ConvHalfCpltCallback()` (while the DMA is filling the second half).
  - Read from the second half of the buffer inside `HAL_ADC_ConvCpltCallback()` (while the DMA is filling the first half).
  This guarantees that the CPU always processes a physically uniform snapshot of the sensor bar.

### Optimization 4: Exponential Moving Average (EMA) Noise Filtering
Standard moving averages require shifting arrays of raw sensor data, consuming CPU and memory.
- **Action**: Use a Single-Pole IIR (EMA) filter which requires only **one float variable** per sensor and runs in a single line of math:
  $$\text{filtered} = (\alpha \times \text{raw}) + ((1.0\text{f} - \alpha) \times \text{filtered})$$
  Set $\alpha = 0.25$ or $0.125$ for extremely fast bit-shift scaling.

### Optimization 5: Slope Feed-Forward (Gravity Compensation)
When climbing a 22-degree incline, gravity acts as a massive 37% backward load. Normal PID must drift off-speed before the error registers and the motors compensate.
- **Action**: Read the pitch angle $\theta$ from the IMU via I2C DMA. Inject a Feed-Forward term to the motor PWM:
  $$\text{Motor\_PWM} = \text{PID\_Output} + (K_{\text{slope}} \times \sin(\theta))$$
  The instant the IMU feels the incline, the robot adds throttle *before* it has a chance to slow down.

### Optimization 6: Decoupled Speed & Active H-Bridge Braking
To execute sharp 90-degree turns at high speed without drifting off the track:
- **Feed-Forward Speed Braking**: Dynamically scale down the base speed when the line error is high:
  $$\text{Target\_Speed} = \text{Base\_Speed} - (K_{\text{brake}} \times |\text{steering\_output}|)$$
- **Active Brake**: Standard drivers coast when PWM is 0. Implement active braking by shorting the H-Bridge terminals (`IN1=1, IN2=1`) for 20ms to lock the wheels and pivot the robot instantly.

### Optimization 7: Centroid Line Calculations
Do not use binary state logic (e.g. "sensor 3 is active"). Use a weighted centroid to find the exact sub-millimeter position of the line:
  $$\text{LinePosition} = \frac{\sum (\text{Normalized\_Value}[i] \times \text{Sensor\_Distance}[i])}{\sum \text{Normalized\_Value}[i]}$$
  Calibrate the sensors at boot by spinning the robot in place for 3 seconds to map raw $ADC_{\text{min}}$ and $ADC_{\text{max}}$ values, keeping the normalization robust in any lighting.

---

## 4. Hardware Pin Mapping & Physical Layout

**Mecanum Wheel Motor Mapping:**
* `M1` = Top Right (Front-Right, `fr`)
* `M2` = Bottom Right (Rear-Right, `rr`)
* `M3` = Bottom Left (Rear-Left, `rl`)
* `M4` = Top Left (Front-Left, `fl`)

### Optimization 8: State Machines (Switch vs If-Else)
In embedded programming, particularly when managing robot states (e.g. Test Sequence, Navigation Modes, Error States), using long chains of `if-else if-else` statements is bad practice.
- **Why it's bad:** The CPU must evaluate each condition sequentially ($O(N)$ complexity). This eats up clock cycles, harms readability, and can confuse the CPU branch predictor, causing pipeline stalls.
- **The Alternative:** Use a `switch` statement! The compiler optimizes `switch` statements into **Jump Tables**. The CPU evaluates the expression once, instantly calculates the memory address of the matching block, and jumps straight there ($O(1)$ complexity). Always prefer `switch` statements or array-of-function-pointers for hot-path state machines.
