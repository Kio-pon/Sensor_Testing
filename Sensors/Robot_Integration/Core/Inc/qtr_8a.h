#ifndef QTR_8A_H
#define QTR_8A_H

#include "main.h"

#define MAX_QTR_SENSORS 8
#define QTR_DEFAULT_THRESHOLD 2000 // For 12-bit ADC (range 0-4095)

/**
 * @brief Configurable ADC channel with its specific ADC handle
 */
typedef struct {
    ADC_HandleTypeDef *hadc;            /**< Pointer to specific ADC handle (ADC1/ADC2/ADC3/ADC4) */
    uint32_t channel;                   /**< Configured ADC channel */
} QTR_Sensor_t;

/**
 * @brief Flexible QTR Sensor Array structure with per-sensor ADC configuration
 */
typedef struct {
    QTR_Sensor_t sensors[MAX_QTR_SENSORS]; /**< List of sensor configs */
    uint8_t num_sensors;                   /**< Number of active sensors */
    uint16_t threshold;                    /**< Brightness threshold */
} QTR_Array_t;

/**
 * @brief Initializes a QTR Array instance with explicit per-sensor ADC mappings
 */
void QTR_Init(QTR_Array_t *array, ADC_HandleTypeDef **hadc_list, const uint32_t *channels, uint8_t num_sensors, uint16_t threshold);

/**
 * @brief Reads raw analog values for the QTR Array
 * @param sensor_values Array to populate with raw 12-bit ADC values [0, 4095]
 */
void QTR_ReadRaw(QTR_Array_t *array, uint16_t *sensor_values);

/**
 * @brief Reads the digital bitmask state of the array
 * @return uint8_t Bitmask state. Bit i is set to 1 if sensor i detects the black line (value >= threshold)
 */
uint8_t QTR_ReadDigital(QTR_Array_t *array);

/**
 * @brief Calculates the weighted-average line position from the front array
 * @param sensor_values Pointer to raw sensor values array
 * @return float Calculated position centered around 0 (e.g., -3500 to +3500)
 */
float QTR_GetLinePosition(QTR_Array_t *array, uint16_t *sensor_values);

#endif /* QTR_8A_H */

