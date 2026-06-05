#include "sharp_ir.h"
#include <math.h>

uint16_t sharp_ir_raw = 0;

void Sharp_Poll(ADC_HandleTypeDef* hadc)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    
    // PC3 is mapped to ADC1_IN9
    sConfig.Channel = ADC_CHANNEL_9;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5; // Matching your CubeMX setup
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
        return;
    }
    
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 2) == HAL_OK) {
        sharp_ir_raw = HAL_ADC_GetValue(hadc);
    }
    HAL_ADC_Stop(hadc);
}

float Sharp_GetDistanceCM(void) {
    if (sharp_ir_raw < 100) return 100.0f; // Max distance / out of bounds
    
    // Generic Sharp IR formula for 12-bit ADC at 3.3V
    // Distance (cm) = 27.86 * (Voltage)^-1.15
    float voltage = ((float)sharp_ir_raw * 3.3f) / 4095.0f;
    if (voltage < 0.1f) return 100.0f;
    
    return 27.86f / powf(voltage, 1.15f);
}
