/**
  ******************************************************************************
  * @file           : sharp_sensor.h
  * @brief          : Modular driver for Sharp IR Distance Sensors (e.g. GP2Y0A02YK)
  ******************************************************************************
  */
#ifndef __SHARP_SENSOR_H
#define __SHARP_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/**
  * @brief Sharp Sensor Handle Structure
  */
typedef struct {
    ADC_HandleTypeDef *hadc;    /* Pointer to ADC handle */
    uint32_t channel;           /* ADC Channel (e.g., ADC_CHANNEL_1) */
} SharpSensor_t;

/* Function Prototypes -------------------------------------------------------*/

/**
  * @brief Initializes the Sharp sensor instance
  * @param sensor Pointer to sensor handle
  * @param hadc Pointer to HAL ADC handle
  * @param channel ADC channel connected to the sensor
  */
void SharpSensor_Init(SharpSensor_t *sensor, ADC_HandleTypeDef *hadc, uint32_t channel);

/**
  * @brief Reads raw ADC value from the sensor
  * @param sensor Pointer to sensor handle
  * @retval 12-bit ADC value (0-4095)
  */
uint32_t SharpSensor_ReadRaw(SharpSensor_t *sensor);

/**
  * @brief Reads voltage from the sensor
  * @param sensor Pointer to sensor handle
  * @retval Voltage in Volts (0.0 - 3.3)
  */
float SharpSensor_ReadVoltage(SharpSensor_t *sensor);

/**
  * @brief Calculates distance in centimeters
  * @param sensor Pointer to sensor handle
  * @retval Distance in cm
  */
float SharpSensor_ReadDistance(SharpSensor_t *sensor);

#ifdef __cplusplus
}
#endif

#endif /* __SHARP_SENSOR_H */
