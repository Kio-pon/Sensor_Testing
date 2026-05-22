#include "qtr_8a.h"
#include <string.h>

void QTR_Init(QTR_Array_t *array, ADC_HandleTypeDef **hadc_list, const uint32_t *channels, uint8_t num_sensors, uint16_t threshold)
{
    if (array == NULL || hadc_list == NULL || channels == NULL) return;
    
    array->num_sensors = (num_sensors > MAX_QTR_SENSORS) ? MAX_QTR_SENSORS : num_sensors;
    array->threshold = (threshold == 0) ? QTR_DEFAULT_THRESHOLD : threshold;
    
    // Copy the channel and ADC mappings
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        array->sensors[i].hadc = hadc_list[i];
        array->sensors[i].channel = channels[i];
        
        // Trigger self-calibration for this ADC instance if not already done
        if (hadc_list[i] != NULL) {
            HAL_ADCEx_Calibration_Start(hadc_list[i], ADC_SINGLE_ENDED);
        }
    }
}

void QTR_ReadRaw(QTR_Array_t *array, uint16_t *sensor_values)
{
    if (array == NULL || sensor_values == NULL) return;
    
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5; // High accuracy sampling
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        if (array->sensors[i].hadc == NULL) {
            sensor_values[i] = 0;
            continue;
        }
        
        sConfig.Channel = array->sensors[i].channel;
        
        // Configure ADC to scan this specific channel
        if (HAL_ADC_ConfigChannel(array->sensors[i].hadc, &sConfig) != HAL_OK) {
            sensor_values[i] = 0;
            continue;
        }
        
        // Start Conversion
        if (HAL_ADC_Start(array->sensors[i].hadc) != HAL_OK) {
            sensor_values[i] = 0;
            continue;
        }
        
        // Wait for completion (Timeout = 10ms)
        if (HAL_ADC_PollForConversion(array->sensors[i].hadc, 10) == HAL_OK) {
            sensor_values[i] = HAL_ADC_GetValue(array->sensors[i].hadc);
        } else {
            sensor_values[i] = 0; // Timeout error
        }
        
        HAL_ADC_Stop(array->sensors[i].hadc);
    }
}

uint8_t QTR_ReadDigital(QTR_Array_t *array)
{
    if (array == NULL) return 0;
    
    uint16_t raw_values[MAX_QTR_SENSORS];
    QTR_ReadRaw(array, raw_values);
    
    uint8_t digital_state = 0;
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        if (raw_values[i] >= array->threshold) {
            digital_state |= (1 << i);
        }
    }
    
    return digital_state;
}

float QTR_GetLinePosition(QTR_Array_t *array, uint16_t *sensor_values)
{
    if (array == NULL || sensor_values == NULL) return 0.0f;
    
    uint32_t sum = 0;
    uint32_t weighted_sum = 0;
    
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        uint16_t val = sensor_values[i];
        
        // For black line tracking (darker values are higher)
        // Only count values above a minimal noise baseline (e.g. 200)
        if (val > 200) {
            sum += val;
            weighted_sum += (uint32_t)val * i * 1000;
        }
    }
    
    // If no line is detected anywhere, return the target (centered = index middle)
    if (sum == 0) {
        return (float)((array->num_sensors - 1) * 500);
    }
    
    // Center position calculations:
    // e.g. for 8 sensors, indices are 0 to 7. 
    // Position range: 0 to 7000. Center is 3500.
    return (float)weighted_sum / (float)sum;
}

