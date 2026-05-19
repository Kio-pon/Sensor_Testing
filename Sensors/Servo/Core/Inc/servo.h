/* servo.h - Hobby Servo Driver for STM32 HAL
 *
 * Assumes timer configured at 1MHz tick rate (Prescaler=47 @ 48MHz):
 *   - Period = 19999  ->  50Hz (20ms) PWM
 *   - Pulse  = 1000   ->    0 degrees (1ms)
 *   - Pulse  = 1500   ->   90 degrees (1.5ms)
 *   - Pulse  = 2000   ->  180 degrees (2ms)
 */

#ifndef SERVO_H
#define SERVO_H

#include "stm32f3xx_hal.h"

/* ---- Configuration ---- */
#define SERVO_PULSE_MIN   1000U   /* 1000us  ->   0 degrees */
#define SERVO_PULSE_MAX   2000U   /* 2000us  -> 180 degrees */
#define SERVO_PULSE_MID   1500U   /* 1500us  ->  90 degrees */

/* ---- Servo handle ---- */
typedef struct
{
    TIM_HandleTypeDef *htim;     /* Pointer to HAL timer handle (TIM2, etc.) */
    uint32_t           channel;  /* Timer channel, e.g. TIM_CHANNEL_1         */
    uint32_t           pulse_us; /* Current pulse width in microseconds        */
} Servo_t;

/* ---- API ---- */

/**
 * @brief  Initialise servo and start PWM output.
 * @param  servo   Pointer to Servo_t handle (htim + channel must be set).
 * @param  start_deg  Initial angle in degrees [0–180].
 */
void Servo_Init(Servo_t *servo, uint8_t start_deg);

/**
 * @brief  Set servo angle.
 * @param  servo   Pointer to Servo_t handle.
 * @param  degrees Desired angle [0–180].
 */
void Servo_SetAngle(Servo_t *servo, uint8_t degrees);

/**
 * @brief  Set servo pulse width directly in microseconds.
 * @param  servo    Pointer to Servo_t handle.
 * @param  pulse_us Pulse width in us [SERVO_PULSE_MIN – SERVO_PULSE_MAX].
 */
void Servo_SetPulse(Servo_t *servo, uint32_t pulse_us);

/**
 * @brief  Return current angle (degrees) reconstructed from stored pulse.
 * @param  servo   Pointer to Servo_t handle.
 * @retval Angle in degrees [0–180].
 */
uint8_t Servo_GetAngle(const Servo_t *servo);

/**
 * @brief  Stop PWM output (holds last position mechanically).
 * @param  servo   Pointer to Servo_t handle.
 */
void Servo_Stop(Servo_t *servo);

#endif /* SERVO_H */
