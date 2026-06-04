/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MEMS_INT3_Pin GPIO_PIN_4
#define MEMS_INT3_GPIO_Port GPIOE
#define MEMS_INT4_Pin GPIO_PIN_5
#define MEMS_INT4_GPIO_Port GPIOE
#define OSC32_IN_Pin GPIO_PIN_14
#define OSC32_IN_GPIO_Port GPIOC
#define OSC32_OUT_Pin GPIO_PIN_15
#define OSC32_OUT_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOF
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOF
#define RIGHT_QTR_1_Pin GPIO_PIN_0
#define RIGHT_QTR_1_GPIO_Port GPIOC
#define RIGHT_QTR_2_Pin GPIO_PIN_1
#define RIGHT_QTR_2_GPIO_Port GPIOC
#define ARDUINO_START_Pin GPIO_PIN_2
#define ARDUINO_START_GPIO_Port GPIOF
#define LEFT_QTR_1_Pin GPIO_PIN_0
#define LEFT_QTR_1_GPIO_Port GPIOA
#define F0_Pin GPIO_PIN_1
#define F0_GPIO_Port GPIOA
#define F1_Pin GPIO_PIN_2
#define F1_GPIO_Port GPIOA
#define F2_Pin GPIO_PIN_3
#define F2_GPIO_Port GPIOA
#define F3_Pin GPIO_PIN_4
#define F3_GPIO_Port GPIOF
#define F4_Pin GPIO_PIN_4
#define F4_GPIO_Port GPIOA
#define RIGHT_QTR_3_Pin GPIO_PIN_5
#define RIGHT_QTR_3_GPIO_Port GPIOC
#define RIGHT_QTR_5_Pin GPIO_PIN_1
#define RIGHT_QTR_5_GPIO_Port GPIOB
#define RIGHT_QTR_6_Pin GPIO_PIN_2
#define RIGHT_QTR_6_GPIO_Port GPIOB
#define Sharp_Pin GPIO_PIN_14
#define Sharp_GPIO_Port GPIOB
#define F5_Pin GPIO_PIN_9
#define F5_GPIO_Port GPIOE
#define LD5_Pin GPIO_PIN_10
#define LD5_GPIO_Port GPIOE
#define LD7_Pin GPIO_PIN_11
#define LD7_GPIO_Port GPIOE
#define LD9_Pin GPIO_PIN_12
#define LD9_GPIO_Port GPIOE
#define F6_Pin GPIO_PIN_14
#define F6_GPIO_Port GPIOB
#define F7_Pin GPIO_PIN_15
#define F7_GPIO_Port GPIOB
#define FL_DIR_1_Pin GPIO_PIN_8
#define FL_DIR_1_GPIO_Port GPIOD
#define FL_DIR_2_Pin GPIO_PIN_9
#define FL_DIR_2_GPIO_Port GPIOD
#define ARDUINO_DONE_Pin GPIO_PIN_11
#define ARDUINO_DONE_GPIO_Port GPIOD
#define GYRO_CS_Pin GPIO_PIN_12
#define GYRO_CS_GPIO_Port GPIOD
#define FL_ENC_B_Pin GPIO_PIN_15
#define FL_ENC_B_GPIO_Port GPIOD
#define FL_ENC_A_Pin GPIO_PIN_6
#define FL_ENC_A_GPIO_Port GPIOC
#define RL_ENC_B_Pin GPIO_PIN_7
#define RL_ENC_B_GPIO_Port GPIOC
#define RL_ENC_A_Pin GPIO_PIN_8
#define RL_ENC_A_GPIO_Port GPIOC
#define RR_ENC_B_Pin GPIO_PIN_9
#define RR_ENC_B_GPIO_Port GPIOC
#define RR_ENC_A_Pin GPIO_PIN_8
#define RR_ENC_A_GPIO_Port GPIOA
#define FR_ENC_A_Pin GPIO_PIN_9
#define FR_ENC_A_GPIO_Port GPIOA
#define FR_ENC_B_Pin GPIO_PIN_10
#define FR_ENC_B_GPIO_Port GPIOA
#define DM_Pin GPIO_PIN_11
#define DM_GPIO_Port GPIOA
#define DP_Pin GPIO_PIN_12
#define DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define RR_DIR_2_Pin GPIO_PIN_12
#define RR_DIR_2_GPIO_Port GPIOC
#define RR_DIR_1_Pin GPIO_PIN_0
#define RR_DIR_1_GPIO_Port GPIOD
#define FR_DIR_2_Pin GPIO_PIN_1
#define FR_DIR_2_GPIO_Port GPIOD
#define FR_DIR_1_Pin GPIO_PIN_2
#define FR_DIR_1_GPIO_Port GPIOD
#define FR_PWM_Pin GPIO_PIN_3
#define FR_PWM_GPIO_Port GPIOD
#define RR_PWM_Pin GPIO_PIN_4
#define RR_PWM_GPIO_Port GPIOD
#define MOTOR_STBY_Pin GPIO_PIN_5
#define MOTOR_STBY_GPIO_Port GPIOD
#define FL_PWM_Pin GPIO_PIN_6
#define FL_PWM_GPIO_Port GPIOD
#define RL_PWM_Pin GPIO_PIN_7
#define RL_PWM_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define RL_DIR_2_Pin GPIO_PIN_4
#define RL_DIR_2_GPIO_Port GPIOB
#define RL_DIR_1_Pin GPIO_PIN_5
#define RL_DIR_1_GPIO_Port GPIOB
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB
#define MEMS_INT1_Pin GPIO_PIN_0
#define MEMS_INT1_GPIO_Port GPIOE
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
