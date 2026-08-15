#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

/* ============================================================
   HARDWARE SWITCHBOARD
   1 = compiled and run. 0 = stripped from the binary.
   ============================================================ */
#define ENABLE_CHASSIS       1
#define ENABLE_ENCODERS      1
#define ENABLE_GYRO          1

/* Sensor suite toggles */
#define ENABLE_QTR_ARRAY     1
#define ENABLE_BFD_ARRAY     0
#define ENABLE_SHARP_IR      1

/* Telemetry blocks the CPU over UART. 1 for the bench only.
   ALWAYS set to 0 for a scored run. */
#define ENABLE_TELEMETRY     1

/* ============================================================
   PHYSICAL CONSTANTS (one place, never retype a raw number)
   ============================================================ */
#define WHEEL_DIAMETER_MM    80.0f
#define ENCODER_PPR          11.0f
#define GEAR_RATIO           19.7f
#define TICKS_PER_WHEEL_REV  (ENCODER_PPR * 4.0f * GEAR_RATIO) /* ~866.8 */

/* Derived travel constants. Verify TICKS_PER_CM on the arena
   by driving a known distance and reading the encoder. */
#define WHEEL_CIRC_CM        (3.14159265f * WHEEL_DIAMETER_MM / 10.0f) /* ~25.13 */
#define TICKS_PER_CM         (TICKS_PER_WHEEL_REV / WHEEL_CIRC_CM)     /* ~34.5 */

/* Arena grid : one box is 12 inches = 30.48 cm. */
#define BOX_CM               30.48f
#define TICKS_PER_BOX        ((int32_t)(BOX_CM * TICKS_PER_CM))        /* ~1051 */

/* Ramp : official arena says 22 degrees. 5-12-13 triple gives a
   13 inch hypotenuse = 33.02 cm. MEASURE on the real ramp. */
#define RAMP_HYP_CM          33.02f
#define TICKS_RAMP           ((int32_t)(RAMP_HYP_CM * TICKS_PER_CM))   /* ~1139 */

/* TURN ticks must be measured, not computed. Track width and
   roller slip make the formula unreliable. Placeholder only. */
#define TICKS_TURN_90        0 /* MEASURE on the arena */
#define TICKS_TURN_45        0 /* MEASURE on the arena */

/* Sharp sensor safe floor. Below this it goes blind. Hand off
   to encoders at this distance and never trust Sharp under it. */
#define SHARP_BLIND_CM       25.0f

/* Typical mounting distance offset for Sharp distance sensors.
   Often recessed by the blind distance (e.g., 15cm back from the bumper)
   or extended 5-6cm forward depending on the chassis design. 
   Set to 5.0f as a standard forward offset placeholder. */
#define SHARP_MOUNT_OFFSET_CM 5.0f

/* Sensor fusion weights for the strafe heading correction. */
#define WEIGHT_QTR_OMEGA     0.95f
#define WEIGHT_SHARP_OMEGA   0.05f

/* Control / filter tuning */
#define EMA_ALPHA            0.25f
#define LINE_CENTER          3500.0f /* 8-sensor center target */
#define LINE_DEADBAND        30.0f

/* Deceleration Travel Calibration */
#define TARGET_STRAIGHT_CM     103.0f
#define DECEL_COMPENSATION_CM   12.0f

/* Strafe Deceleration Calibration */
#define EXPECTED_JUNCTION_CM   45.72f
#define STRAFE_DECEL_START_CM  30.0f
#define STRAFE_MIN_SPEED       1000

#define STRAFE_CORRECTION_DEG   -1.5f
#define STRAFE_TICKS_SCALE      0.33f

#endif /* ROBOT_CONFIG_H */
