#ifndef L298N_H
#define L298N_H

#include "stm32f3xx_hal.h"

/* 
 * L298N Motor Object Structure (Class equivalent in C)
 * Each instance of this struct represents one motor connected to half of an L298N.
 */
typedef struct {
    // Direction Control Pins
    GPIO_TypeDef *IN1_Port;
    uint16_t IN1_Pin;
    
    GPIO_TypeDef *IN2_Port;
    uint16_t IN2_Pin;
    
    // PWM Control
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    
    // Configuration
    uint32_t max_pwm;  // Maximum value for PWM (usually ARR value, e.g., 4799)
} L298N_Motor_t;

/* Method declarations */

/**
 * @brief Initializes the motor (Starts the PWM)
 * @param motor Pointer to the motor object instance
 */
void L298N_Motor_Init(L298N_Motor_t *motor);

/**
 * @brief Sets the speed and direction of the motor
 * @param motor Pointer to the motor object instance
 * @param speed Speed from -max_pwm to +max_pwm. Positive is forward, Negative is backward.
 */
void L298N_Motor_SetSpeed(L298N_Motor_t *motor, int32_t speed);

/**
 * @brief Stops the motor (Coasts)
 * @param motor Pointer to the motor object instance
 */
void L298N_Motor_Stop(L298N_Motor_t *motor);

#endif /* L298N_H */
