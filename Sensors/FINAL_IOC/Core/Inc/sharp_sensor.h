#ifndef SHARP_SENSOR_H
#define SHARP_SENSOR_H

#include "main.h"

/**
 * @brief Sharp IR Distance Sensor structure
 */
typedef struct {
    ADC_HandleTypeDef *hadc;    /**< Pointer to ADC handle */
    uint32_t channel;           /**< Configured ADC channel */
} SharpSensor_t;

/**
 * @brief Initializes a Sharp IR sensor instance
 */
void SharpSensor_Init(SharpSensor_t *sensor, ADC_HandleTypeDef *hadc, uint32_t channel);

/**
 * @brief Reads raw 12-bit ADC value from the sensor
 */
uint32_t SharpSensor_ReadRaw(SharpSensor_t *sensor);

/**
 * @brief Reads voltage from the sensor (assuming 3.3V VRef and 12-bit resolution)
 */
float SharpSensor_ReadVoltage(SharpSensor_t *sensor);

/**
 * @brief Calculates distance in centimeters (optimized for GP2Y0A02YK or similar 20-150cm Sharp sensors)
 */
float SharpSensor_ReadDistance(SharpSensor_t *sensor);

#endif /* SHARP_SENSOR_H */
