#ifndef QTR_8A_H
#define QTR_8A_H

#include "main.h"

#define MAX_QTR_SENSORS 8
#define QTR_DEFAULT_TIMEOUT_US 2500

/**
 * @brief Configurable GPIO pin for QTR-RC
 */
typedef struct {
    GPIO_TypeDef *Port;                 /**< Pointer to GPIO Port */
    uint16_t Pin;                       /**< Configured GPIO Pin */
} QTR_Pin_t;

/**
 * @brief Flexible QTR-RC Sensor Array structure
 */
typedef struct {
    QTR_Pin_t sensors[MAX_QTR_SENSORS];            /**< List of sensor configs */
    uint8_t num_sensors;                           /**< Number of active sensors */
    uint32_t timeout_us;                           /**< Timeout for RC discharge (us) */
    uint16_t threshold;                            /**< Brightness threshold (scaled 0-1000) */
    uint16_t calibrated_minimums[MAX_QTR_SENSORS]; /**< Calibrated minimums (light/white in us) */
    uint16_t calibrated_maximums[MAX_QTR_SENSORS]; /**< Calibrated maximums (dark/black in us) */
    uint8_t is_calibrated;                         /**< Calibration status flag */
} QTR_Array_t;

/**
 * @brief Initializes a QTR Array instance with explicit GPIO pins
 */
void QTR_Init(QTR_Array_t *array, GPIO_TypeDef **ports, const uint16_t *pins, uint8_t num_sensors, uint32_t timeout_us);

/**
 * @brief Performs an interactive manual sweeping calibration sequence over serial terminal.
 */
void QTR_CalibrateSensorSweep(QTR_Array_t *array, volatile uint16_t *dma_buffer, uint32_t duration_ms, void *chassis);

/**
 * @brief Calibrates all three QTR arrays simultaneously using standard strafing at 20% speed
 */
void QTR_CalibrateAllThree(
    QTR_Array_t *front, volatile uint16_t *front_dma,
    QTR_Array_t *left, volatile uint16_t *left_dma,
    QTR_Array_t *right, volatile uint16_t *right_dma,
    uint32_t duration_ms, void *chassis, void (*poll_callback)(void));

/**
 * @brief Reads raw analog values (discharge time in microseconds) for the QTR Array
 * @param sensor_values Array to populate with raw times [0, timeout_us]
 */
void QTR_ReadRaw(QTR_Array_t *array, uint16_t *sensor_values);

/**
 * @brief Reads calibrated (normalized) values scaled between [0, 1000] based on min/max
 * @param array Pointer to QTR_Array_t
 * @param calibrated_values Output array populated with [0, 1000] values
 * @param dma_buffer Raw DMA buffer to read from (kept for API compatibility, ignored internally)
 */
void QTR_ReadCalibrated(QTR_Array_t *array, uint16_t *calibrated_values, volatile uint16_t *dma_buffer);

/**
 * @brief Reads the digital bitmask state of the array based on scaled threshold
 */
uint8_t QTR_ReadDigital(QTR_Array_t *array);

/**
 * @brief Calculates the weighted-average line position from the array
 * @param sensor_values Pointer to calibrated sensor values array
 * @return float Calculated position
 */
float QTR_GetLinePosition(QTR_Array_t *array, uint16_t *sensor_values);

#endif /* QTR_8A_H */
