#include "motor_driver.h"
#include <stddef.h>

void Motor_Init(TB6612_Motor_t *motor)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // Start PWM output on the specified timer channel
    HAL_TIM_PWM_Start(motor->htim, motor->channel);
    
    // Ensure direction inputs are reset
    Motor_Stop(motor);
}

void Motor_SetSpeed(TB6612_Motor_t *motor, int32_t speed)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // Clamp speed limits
    if (speed > (int32_t)motor->max_pwm) speed = (int32_t)motor->max_pwm;
    if (speed < -(int32_t)motor->max_pwm) speed = -(int32_t)motor->max_pwm;
    
    if (speed > 0) {
        // Forward rotation
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint32_t)speed);
    } 
    else if (speed < 0) {
        // Reverse rotation
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint32_t)(-speed));
    } 
    else {
        // Stop
        Motor_Brake(motor);
    }
}

void Motor_Brake(TB6612_Motor_t *motor)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // Short Brake: Pull both IN1 and IN2 HIGH to short-circuit windings
    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, motor->max_pwm); // High braking duty
}

void Motor_Stop(TB6612_Motor_t *motor)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // Coast Stop: Pull both direction pins LOW
    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
}

void Chassis_Init(Mecanum_Chassis_t *chassis)
{
    if (chassis == NULL) return;
    
    Motor_Init(&chassis->fl);
    Motor_Init(&chassis->fr);
    Motor_Init(&chassis->rl);
    Motor_Init(&chassis->rr);
    
    // Pull standby HIGH to enable drivers initially
    Chassis_SetStandby(chassis, 1);
}

void Chassis_SetStandby(Mecanum_Chassis_t *chassis, uint8_t state)
{
    if (chassis == NULL || chassis->STBY_Port == NULL) return;
    
    HAL_GPIO_WritePin(chassis->STBY_Port, chassis->STBY_Pin, 
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Chassis_Drive(Mecanum_Chassis_t *chassis, int32_t vx, int32_t vy, int32_t omega)
{
    if (chassis == NULL) return;
    
    // Mecanum Kinematic formulas
    int32_t fl_speed = vx + vy - omega;
    int32_t fr_speed = vx - vy + omega;
    int32_t rl_speed = vx - vy - omega;
    int32_t rr_speed = vx + vy + omega;
    
    Motor_SetSpeed(&chassis->fl, fl_speed);
    Motor_SetSpeed(&chassis->fr, fr_speed);
    Motor_SetSpeed(&chassis->rl, rl_speed);
    Motor_SetSpeed(&chassis->rr, rr_speed);
}

void Chassis_BrakeAll(Mecanum_Chassis_t *chassis)
{
    if (chassis == NULL) return;
    
    Motor_Brake(&chassis->fl);
    Motor_Brake(&chassis->fr);
    Motor_Brake(&chassis->rl);
    Motor_Brake(&chassis->rr);
}
