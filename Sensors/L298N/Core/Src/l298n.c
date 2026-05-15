#include "l298n.h"

void L298N_Motor_Init(L298N_Motor_t *motor)
{
    // Basic null-pointer safety check
    if (motor == NULL || motor->htim == NULL) return;
    
    // Start the PWM generation for this specific motor's channel
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
    
    // Ensure the motor is initially stopped
    L298N_Motor_Stop(motor);
}

void L298N_Motor_SetSpeed(L298N_Motor_t *motor, int32_t speed)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // Clamp the speed to the defined max_pwm limits
    if (speed > (int32_t)motor->max_pwm) speed = motor->max_pwm;
    if (speed < -(int32_t)motor->max_pwm) speed = -motor->max_pwm;
    
    if (speed > 0) {
        // Forward
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint32_t)speed);
    } 
    else if (speed < 0) {
        // Reverse
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint32_t)(-speed));
    } 
    else {
        // Stop
        L298N_Motor_Stop(motor);
    }
}

void L298N_Motor_Stop(L298N_Motor_t *motor)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // Set both direction pins to LOW to coast the motor
    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
    
    // Set PWM duty cycle to 0
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
}
