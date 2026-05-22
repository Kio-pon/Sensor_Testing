#include "sharp_sensor.h"

void SharpSensor_Init(SharpSensor_t *sensor, ADC_HandleTypeDef *hadc, uint32_t channel)
{
    if (sensor == NULL || hadc == NULL) return;
    
    sensor->hadc = hadc;
    sensor->channel = channel;
}

uint32_t SharpSensor_ReadRaw(SharpSensor_t *sensor)
{
    if (sensor == NULL || sensor->hadc == NULL) return 0;
    
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = sensor->channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5; // Stable sampling time
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    
    if (HAL_ADC_ConfigChannel(sensor->hadc, &sConfig) != HAL_OK) {
        return 0;
    }
    
    uint32_t raw_value = 0;
    if (HAL_ADC_Start(sensor->hadc) == HAL_OK) {
        if (HAL_ADC_PollForConversion(sensor->hadc, 10) == HAL_OK) {
            raw_value = HAL_ADC_GetValue(sensor->hadc);
        }
        HAL_ADC_Stop(sensor->hadc);
    }
    
    return raw_value;
}

float SharpSensor_ReadVoltage(SharpSensor_t *sensor)
{
    uint32_t raw = SharpSensor_ReadRaw(sensor);
    return (float)raw * 3.3f / 4095.0f;
}

float SharpSensor_ReadDistance(SharpSensor_t *sensor)
{
    float voltage = SharpSensor_ReadVoltage(sensor);
    
    // Limits checking for sensor detection range (20cm to 150cm)
    if (voltage < 0.35f) return 150.0f; 
    if (voltage > 2.6f)  return 15.0f; 
    
    // Linearization formula for standard GP2Y0A02YK distance sensor
    float distance = 62.28f / (voltage - 0.02f);
    
    return distance;
}
