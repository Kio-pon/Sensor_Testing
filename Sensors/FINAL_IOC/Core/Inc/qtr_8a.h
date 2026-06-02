#ifndef QTR_8A_H
#define QTR_8A_H

#include "main.h"

#define MAX_QTR_SENSORS 8

/**
 * @brief Flexible QTR-A Sensor Array structure (Analog)
 */
typedef struct {
    uint8_t num_sensors;                           /**< Number of active sensors */
    uint16_t threshold;                            /**< Brightness threshold (scaled 0-1000) */
    uint16_t calibrated_minimums[MAX_QTR_SENSORS]; /**< Calibrated minimums (light/white in ADC) */
    uint16_t calibrated_maximums[MAX_QTR_SENSORS]; /**< Calibrated maximums (dark/black in ADC) */
    uint8_t is_calibrated;                         /**< Calibration status flag */
} QTR_Array_t;

void QTR_Init(QTR_Array_t *array, uint8_t num_sensors);
void QTR_ReadRaw(QTR_Array_t *array, uint16_t *sensor_values, volatile uint16_t *adc_buffer);
void QTR_ReadCalibrated(QTR_Array_t *array, uint16_t *calibrated_values, volatile uint16_t *adc_buffer);
float QTR_GetLinePosition(QTR_Array_t *array, uint16_t *sensor_values);
void QTR_CalibrateAllThree(
    QTR_Array_t *front, volatile uint16_t *front_adc,
    QTR_Array_t *left, volatile uint16_t *left_adc,
    QTR_Array_t *right, volatile uint16_t *right_adc,
    uint32_t duration_ms, void *chassis, void (*poll_callback)(void));

#endif
