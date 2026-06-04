/**
  ******************************************************************************
  * @file           : sharp_sensor.c
  * @brief          : Modular driver for Sharp IR Distance Sensors
  ******************************************************************************
  */
#include "sharp_sensor.h"

volatile float sharp_cm_front = 150.0f;
volatile float sharp_cm_left  = 150.0f;
volatile float sharp_cm_right = 150.0f;

/**
  * @brief Initializes the Sharp sensor instance
  */
void SharpSensor_Init(SharpSensor_t *sensor, ADC_HandleTypeDef *hadc, uint32_t channel) {
    sensor->hadc = hadc;
    sensor->channel = channel;
    sensor->filtered_distance = 150.0f;
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
    
    float new_dist;
    
    // GP2Y0A02YK output is roughly 0.4V at 150cm and 2.5V at 20cm
    if (voltage < 0.35f) new_dist = 150.0f; 
    else if (voltage > 2.51f) new_dist = 25.0f; // 25cm hard threshold
    else {
        new_dist = 62.28f / (voltage - 0.02f);
        if (new_dist < 25.0f) new_dist = 25.0f; // 25cm floor
    }
    
    // Exponential Moving Average (EMA) Filter (Alpha = 0.2)
    // Filters out high-frequency noise while remaining responsive
    sensor->filtered_distance = (0.2f * new_dist) + (0.8f * sensor->filtered_distance);
    
    return sensor->filtered_distance;
}
