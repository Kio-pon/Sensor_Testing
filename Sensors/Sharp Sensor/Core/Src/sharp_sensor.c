/**
  ******************************************************************************
  * @file           : sharp_sensor.c
  * @brief          : Modular driver for Sharp IR Distance Sensors
  ******************************************************************************
  */
#include "sharp_sensor.h"

/**
  * @brief Initializes the Sharp sensor instance
  */
void SharpSensor_Init(SharpSensor_t *sensor, ADC_HandleTypeDef *hadc, uint32_t channel) {
    sensor->hadc = hadc;
    sensor->channel = channel;
}

/**
  * @brief Reads raw ADC value from the sensor
  * Note: Configures the channel before reading to allow multiple sensors on one ADC.
  */
uint32_t SharpSensor_ReadRaw(SharpSensor_t *sensor) {
    ADC_ChannelConfTypeDef sConfig = {0};
    
    sConfig.Channel = sensor->channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5; // Increased sampling time for stability
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    if (HAL_ADC_ConfigChannel(sensor->hadc, &sConfig) != HAL_OK) {
        return 0;
    }

    HAL_ADC_Start(sensor->hadc);
    if (HAL_ADC_PollForConversion(sensor->hadc, 10) == HAL_OK) {
        uint32_t val = HAL_ADC_GetValue(sensor->hadc);
        HAL_ADC_Stop(sensor->hadc);
        return val;
    }
    HAL_ADC_Stop(sensor->hadc);
    return 0;
}

/**
  * @brief Reads voltage from the sensor (assuming 3.3V VCC and 12-bit ADC)
  */
float SharpSensor_ReadVoltage(SharpSensor_t *sensor) {
    uint32_t raw = SharpSensor_ReadRaw(sensor);
    return (float)raw * 3.3f / 4095.0f;
}

/**
  * @brief Calculates distance in centimeters for GP2Y0A02YK (20-150cm)
  */
float SharpSensor_ReadDistance(SharpSensor_t *sensor) {
    float voltage = SharpSensor_ReadVoltage(sensor);
    
    // Safety checks for sensor range
    // GP2Y0A02YK output is roughly 0.4V at 150cm and 2.5V at 20cm
    if (voltage < 0.35f) return 150.0f; 
    if (voltage > 2.6f)  return 15.0f; 
    
    /* 
     * Distance calculation for GP2Y0A02YK:
     * A common approximation for the inverse relationship is:
     * Distance (cm) = 62.28 / (Voltage - 0.02)
     */
    float distance = 62.28f / (voltage - 0.02f);
    
    return distance;
}
