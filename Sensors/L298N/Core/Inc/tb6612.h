#ifndef TB6612_H
#define TB6612_H

#include "stm32f3xx_hal.h"

/**
 * @brief TB6612 Motor Object Structure
 */
typedef struct {
    GPIO_TypeDef *IN1_Port;
    uint16_t IN1_Pin;
    GPIO_TypeDef *IN2_Port;
    uint16_t IN2_Pin;
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    uint32_t max_pwm;
} TB6612_Motor_t;

/* Function Prototypes */

/**
 * @brief Initializes the motor (Starts PWM)
 */
void TB6612_Motor_Init(TB6612_Motor_t *motor);

/**
 * @brief Sets speed and direction (-max_pwm to +max_pwm)
 */
void TB6612_Motor_SetSpeed(TB6612_Motor_t *motor, int32_t speed);

/**
 * @brief Stops the motor (Short Brake mode)
 */
void TB6612_Motor_Stop(TB6612_Motor_t *motor);

/**
 * @brief Controls the Standby pin of the chip
 * @param state 1 for Active, 0 for Standby (Power Save)
 */
void TB6612_Set_Standby(GPIO_TypeDef *STBY_Port, uint16_t STBY_Pin, uint8_t state);

#endif /* TB6612_H */
