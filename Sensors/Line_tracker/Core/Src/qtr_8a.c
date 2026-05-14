#include "qtr_8a.h"

// Local pointer to the ADC handle
static ADC_HandleTypeDef *qtr_adc = NULL;

// The channels we want to scan (PA0 = IN1, PA1 = IN2, PA2 = IN3, PA3 = IN4)
static const uint32_t QTR_CHANNELS[NUM_QTR_SENSORS] = {
    ADC_CHANNEL_1, 
    ADC_CHANNEL_2, 
    ADC_CHANNEL_3, 
    ADC_CHANNEL_4
};

void QTR_Init(ADC_HandleTypeDef *hadc) {
    qtr_adc = hadc;
}

void QTR_Read(uint16_t *sensor_values) {
    if (qtr_adc == NULL) return;

    ADC_ChannelConfTypeDef sConfig = {0};
    
    // Default config that applies to all our channels
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5; // Adjust if needed
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;

    for (int i = 0; i < NUM_QTR_SENSORS; i++) {
        sConfig.Channel = QTR_CHANNELS[i];
        
        // Configure the channel to be the only one in Rank 1
        if (HAL_ADC_ConfigChannel(qtr_adc, &sConfig) != HAL_OK) {
            // Configuration Error
            sensor_values[i] = 0;
            continue;
        }

        // Start ADC Conversion
        if (HAL_ADC_Start(qtr_adc) != HAL_OK) {
            sensor_values[i] = 0;
            continue;
        }

        // Wait for completion (Timeout = 10ms)
        if (HAL_ADC_PollForConversion(qtr_adc, 10) == HAL_OK) {
            // Read value
            sensor_values[i] = HAL_ADC_GetValue(qtr_adc);
        } else {
            sensor_values[i] = 0; // Timeout
        }

        // Stop ADC
        HAL_ADC_Stop(qtr_adc);
    }
}
