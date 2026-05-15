#include "qtr_8a.h"

// Local pointer to the ADC handle
static ADC_HandleTypeDef *qtr_adc = NULL;

// The channels mapped as requested: IN9 (PC3) first, then IN1 (PA0) onwards.
static const uint32_t QTR_CHANNELS[NUM_QTR_SENSORS] = {
    ADC_CHANNEL_9, // PC3 (Sensor 1)
    ADC_CHANNEL_2, // PA0 (Sensor 2)
    ADC_CHANNEL_3, // PA1 (Sensor 3)
    ADC_CHANNEL_4, // PA2 (Sensor 4)
    ADC_CHANNEL_5, // PA3 (Sensor 5)
    ADC_CHANNEL_6, // PA4 (Sensor 6)
    ADC_CHANNEL_7, // PA5 (Sensor 7)
    ADC_CHANNEL_8  // PA6 (Sensor 8)
};

void QTR_Init(ADC_HandleTypeDef *hadc) {
    qtr_adc = hadc;
    HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);
    
}

void QTR_Read(uint16_t *sensor_values) {
    if (qtr_adc == NULL) return;

    ADC_ChannelConfTypeDef sConfig = {0};
    
    // Default config that applies to all our channels
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5; // Increased sampling time for better accuracy
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

uint8_t QTR_ReadDigital(void) {
    uint16_t raw_values[NUM_QTR_SENSORS];
    QTR_Read(raw_values);
    uint8_t sensor_state = 0;
    for (int i = 0; i < NUM_QTR_SENSORS; i++) {
        if (raw_values[i] >= 3500) {
            sensor_state |= (1 << i);
        }
    }
    return sensor_state;
}
