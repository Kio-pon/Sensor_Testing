#ifndef ROBOT_CORE_H
#define ROBOT_CORE_H

#include "main.h"

// ==============================================================================
// 🎛️ THE HARDWARE SWITCHBOARD - TOGGLE MODULES FOR ISOLATED OR COMBINED TESTS
// 1 = Compile and Run. 0 = Completely erase from CPU memory and execution.
// ==============================================================================

// ─── MOVEMENT & ODOMETRY ───
#define ENABLE_CHASSIS       1   // 1 = Motors active, 0 = Motors disabled
#define ENABLE_ENCODERS      1   // 1 = Track wheel ticks in background

// ─── QTR-8A LINE ARRAYS ───
#define ENABLE_QTR_FRONT     1   // Front array (ADC1)
#define ENABLE_QTR_LEFT      0   // Left array (ADC3)
#define ENABLE_QTR_RIGHT     0   // Right array (ADC2)

// ─── SHARP IR DISTANCE SENSORS ───
#define ENABLE_SHARP_FRONT   0   // Front Sharp (ADC1_IN10)
#define ENABLE_SHARP_LEFT    0   // Left Sharp (ADC3_IN8)
#define ENABLE_SHARP_RIGHT   0   // Right Sharp (ADC3_IN9)

// ─── MECHANISMS & AUX SENSORS ───
#define ENABLE_COLOR_SENSOR  0   // TCS34725 I2C Sensor
#define ENABLE_ARM_SERVO     0   // Robotic Arm / Elevator PWM
#define ENABLE_SHOOTER_SERVO 0   // Pallet Shooter PWM

// ─── DEBUGGING ─────────────────
#define ENABLE_TELEMETRY     1   // Print 10Hz dashboard to PuTTY

// ==============================================================================

void Robot_Init(void);
void Robot_RunLoop(void);

#endif /* ROBOT_CORE_H */
