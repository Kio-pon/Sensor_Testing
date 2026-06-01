#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <stdint.h>

/* ============================================================
   HARDWARE SWITCHBOARD
   1 = compiled and run. 0 = stripped from the binary.
   Start with FRONT only, everything else 0, to tune the
   forward line follower first.
   ============================================================ */
#define ENABLE_CHASSIS       1
#define ENABLE_ENCODERS      0

#define ENABLE_QTR_FRONT     1
#define ENABLE_QTR_RIGHT     0
#define ENABLE_QTR_LEFT      0

#define ENABLE_SHARP_FRONT   0
#define ENABLE_SHARP_LEFT    0
#define ENABLE_SHARP_RIGHT   0

#define ENABLE_COLOR_SENSOR  0
#define ENABLE_ARM_SERVO     0
#define ENABLE_LIFT_MOTOR    0

/* LED bar shares PE8/PE9/PE13 with the LEFT QTR array.
   Keep this 0 whenever ENABLE_QTR_LEFT is 1. */
#define ENABLE_LED_BAR       0

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
#define SHARP_BLIND_CM       15.0f

/* Sensor fusion weights for the strafe heading correction. */
#define WEIGHT_QTR_OMEGA     0.95f
#define WEIGHT_SHARP_OMEGA   0.05f

/* Control / filter tuning */
#define EMA_ALPHA            0.25f
#define LINE_CENTER          3500.0f /* 8-sensor center target */
#define LINE_DEADBAND        50.0f

#endif /* ROBOT_CONFIG_H */
