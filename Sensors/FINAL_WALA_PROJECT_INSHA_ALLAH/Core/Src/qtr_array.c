#include "qtr_array.h"

uint16_t qtr_front[8] = {0};
uint16_t qtr_left[6]  = {0};
uint16_t qtr_right[6] = {0};

// Helper function to dynamically switch the ADC channel
static uint16_t Read_ADC_Channel(ADC_HandleTypeDef* hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5; // Matched to new CubeMX config
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) return 0;
    
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 2) == HAL_OK) {
        uint16_t val = HAL_ADC_GetValue(hadc);
        HAL_ADC_Stop(hadc);
        return val;
    }
    HAL_ADC_Stop(hadc);
    return 0;
}

void QTR_Poll(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, ADC_HandleTypeDef* hadc3, ADC_HandleTypeDef* hadc4)
{
    /* ========================================================
       FRONT QTR-8A (8 Pins)
       PA0, PA1, PA2, PA3, PA4, PC0, PC1, PC2 
       ======================================================== */
    qtr_front[0] = Read_ADC_Channel(hadc1, ADC_CHANNEL_1); // PA0
    qtr_front[1] = Read_ADC_Channel(hadc1, ADC_CHANNEL_2); // PA1
    qtr_front[2] = Read_ADC_Channel(hadc1, ADC_CHANNEL_3); // PA2
    qtr_front[3] = Read_ADC_Channel(hadc1, ADC_CHANNEL_4); // PA3
    qtr_front[4] = Read_ADC_Channel(hadc2, ADC_CHANNEL_1); // PA4
    qtr_front[5] = Read_ADC_Channel(hadc1, ADC_CHANNEL_6); // PC0
    qtr_front[6] = Read_ADC_Channel(hadc1, ADC_CHANNEL_7); // PC1
    qtr_front[7] = Read_ADC_Channel(hadc1, ADC_CHANNEL_8); // PC2

    /* ========================================================
       LEFT QTR-A (6 Pins)
       PD10, PD11, PD13, PD14, PE7, PB0
       ======================================================== */
    qtr_left[0] = Read_ADC_Channel(hadc3, ADC_CHANNEL_7);  // PD10
    qtr_left[1] = Read_ADC_Channel(hadc3, ADC_CHANNEL_8);  // PD11
    qtr_left[2] = Read_ADC_Channel(hadc3, ADC_CHANNEL_10); // PD13
    qtr_left[3] = Read_ADC_Channel(hadc3, ADC_CHANNEL_11); // PD14
    qtr_left[4] = Read_ADC_Channel(hadc3, ADC_CHANNEL_13); // PE7
    qtr_left[5] = Read_ADC_Channel(hadc3, ADC_CHANNEL_12); // PB0

    /* ========================================================
       RIGHT QTR-A (6 Pins)
       PB1, PB12, PB13, PB14, PB15, PF4
       ======================================================== */
    qtr_right[0] = Read_ADC_Channel(hadc3, ADC_CHANNEL_1); // PB1
    qtr_right[1] = Read_ADC_Channel(hadc4, ADC_CHANNEL_3); // PB12
    qtr_right[2] = Read_ADC_Channel(hadc3, ADC_CHANNEL_5); // PB13
    qtr_right[3] = Read_ADC_Channel(hadc4, ADC_CHANNEL_4); // PB14
    qtr_right[4] = Read_ADC_Channel(hadc4, ADC_CHANNEL_5); // PB15
    qtr_right[5] = Read_ADC_Channel(hadc1, ADC_CHANNEL_5); // PF4
}

/* Helper macro for perfectly clean threshold logic */
#define APPLY_QTR_THRESHOLD(val) ( (val) < 4094 ? 0 : (val) )

int32_t QTR_GetFrontLinePosition(void)
{
    uint32_t sum = 0;
    uint32_t weighted_sum = 0;
    
    for(int i = 0; i < 8; i++) {
        uint16_t value = APPLY_QTR_THRESHOLD(qtr_front[i]);
        sum += value;
        weighted_sum += (uint32_t)value * (i * 1000); 
    }
    
    if (sum == 0) return 0; 
    return (weighted_sum / sum) - 3500;
}

int32_t QTR_GetLeftLinePosition(void)
{
    uint32_t sum = 0;
    uint32_t weighted_sum = 0;
    
    for(int i = 0; i < 6; i++) {
        uint16_t value = APPLY_QTR_THRESHOLD(qtr_left[i]);
        sum += value;
        weighted_sum += (uint32_t)value * (i * 1000); 
    }
    
    if (sum == 0) return 0; 
    return (weighted_sum / sum) - 2500;
}

int32_t QTR_GetRightLinePosition(void)
{
    uint32_t sum = 0;
    uint32_t weighted_sum = 0;
    
    for(int i = 0; i < 6; i++) {
        uint16_t value = APPLY_QTR_THRESHOLD(qtr_right[i]);
        sum += value;
        weighted_sum += (uint32_t)value * (i * 1000); 
    }
    
    if (sum == 0) return 0; 
    return (weighted_sum / sum) - 2500;
}
