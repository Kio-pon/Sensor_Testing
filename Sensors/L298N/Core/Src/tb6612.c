#include "tb6612.h"

void TB6612_Motor_Init(TB6612_Motor_t *motor)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // Start PWM for this motor channel
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
    
    // Default to stop
    TB6612_Motor_Stop(motor);
}

void TB6612_Motor_SetSpeed(TB6612_Motor_t *motor, int32_t speed)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // Clamp speed
    if (speed > (int32_t)motor->max_pwm) speed = (int32_t)motor->max_pwm;
    if (speed < -(int32_t)motor->max_pwm) speed = -(int32_t)motor->max_pwm;
    
    if (speed > 0) {
        // Forward (CW)
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint32_t)speed);
    } 
    else if (speed < 0) {
        // Reverse (CCW)
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint32_t)(-speed));
    } 
    else {
        // Stop
        TB6612_Motor_Stop(motor);
    }
}

void TB6612_Motor_Stop(TB6612_Motor_t *motor)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // TB6612 Short Brake: Both IN pins HIGH
    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
    
    // Duty cycle to 0
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
}

void TB6612_Set_Standby(GPIO_TypeDef *STBY_Port, uint16_t STBY_Pin, uint8_t state)
{
    HAL_GPIO_WritePin(STBY_Port, STBY_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
