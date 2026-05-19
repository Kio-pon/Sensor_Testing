/* servo.c - Hobby Servo Driver Implementation for STM32 HAL */

#include "servo.h"

/* ---- Internal helper ---- */

/**
 * @brief  Clamp a value between min and max.
 */
static uint32_t clamp_u32(uint32_t val, uint32_t min, uint32_t max)
{
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

/**
 * @brief  Convert degrees [0-180] to pulse width in microseconds [1000-2000].
 */
static uint32_t deg_to_pulse(uint8_t degrees)
{
    /* Clamp degrees to valid range */
    if (degrees > 180U) degrees = 180U;

    /* Linear map: 0deg -> 1000us, 180deg -> 2000us */
    return SERVO_PULSE_MIN + ((uint32_t)degrees * (SERVO_PULSE_MAX - SERVO_PULSE_MIN)) / 180U;
}

/* ---- Public API ---- */

void Servo_Init(Servo_t *servo, uint8_t start_deg)
{
    if (servo == NULL || servo->htim == NULL) return;

    servo->pulse_us = deg_to_pulse(start_deg);

    /* Start PWM on the configured timer channel */
    HAL_TIM_PWM_Start(servo->htim, servo->channel);

    /* Apply initial pulse width */
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->pulse_us);
}

void Servo_SetAngle(Servo_t *servo, uint8_t degrees)
{
    if (servo == NULL) return;

    servo->pulse_us = deg_to_pulse(degrees);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->pulse_us);
}

void Servo_SetPulse(Servo_t *servo, uint32_t pulse_us)
{
    if (servo == NULL) return;

    servo->pulse_us = clamp_u32(pulse_us, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, servo->pulse_us);
}

uint8_t Servo_GetAngle(const Servo_t *servo)
{
    if (servo == NULL) return 0U;

    /* Inverse of deg_to_pulse */
    uint32_t clamped = clamp_u32(servo->pulse_us, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
    return (uint8_t)(((clamped - SERVO_PULSE_MIN) * 180U) / (SERVO_PULSE_MAX - SERVO_PULSE_MIN));
}

void Servo_Stop(Servo_t *servo)
{
    if (servo == NULL) return;

    HAL_TIM_PWM_Stop(servo->htim, servo->channel);
}
