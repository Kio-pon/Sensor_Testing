#include "qtr_8a.h"
#include <stdio.h>
#include "motor_driver.h"

void QTR_Init(QTR_Array_t *array, uint8_t num_sensors) {
    if (array == NULL) return;
    if (num_sensors > MAX_QTR_SENSORS) num_sensors = MAX_QTR_SENSORS;
    array->num_sensors = num_sensors;
    array->threshold = 500;
    array->is_calibrated = 0;
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        array->calibrated_minimums[i] = 4095; // ADC max
        array->calibrated_maximums[i] = 0;
    }
}

void QTR_ReadRaw(QTR_Array_t *array, uint16_t *sensor_values, volatile uint16_t *adc_buffer) {
    if (array == NULL || sensor_values == NULL || adc_buffer == NULL) return;
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        sensor_values[i] = adc_buffer[i];
    }
}

void QTR_ReadCalibrated(QTR_Array_t *array, uint16_t *calibrated_values, volatile uint16_t *adc_buffer) {
    if (array == NULL || calibrated_values == NULL) return;
    uint16_t sensor_values[MAX_QTR_SENSORS];
    QTR_ReadRaw(array, sensor_values, adc_buffer);
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        // User requested inverted logic: > 3950 is Black, <= 3950 is White
        if (sensor_values[i] > 3950) {
            calibrated_values[i] = 1000; // Black
        } else {
            calibrated_values[i] = 0;    // White
        }
    }
}

float QTR_GetLinePosition(QTR_Array_t *array, uint16_t *sensor_values) {
    if (array == NULL || sensor_values == NULL) return 0.0f;
    uint32_t sum = 0;
    uint32_t weighted_sum = 0;
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        uint16_t val = sensor_values[i];
        if (val > 50) {
            val -= 50;
            sum += val;
            weighted_sum += (uint32_t)val * i * 1000;
        }
    }
    if (sum == 0) return (float)((array->num_sensors - 1) * 500);
    return (float)weighted_sum / (float)sum;
}

void QTR_CalibrateAllThree(
    QTR_Array_t *front, volatile uint16_t *front_adc,
    QTR_Array_t *left, volatile uint16_t *left_adc,
    QTR_Array_t *right, volatile uint16_t *right_adc,
    uint32_t duration_ms, void *chassis, void (*poll_callback)(void)) 
{
    printf("\r\n====================================================\r\n");
    printf("         QTR-8A ANALOG GLOBAL CALIBRATION           \r\n");
    printf("====================================================\r\n");
    
    uint16_t f_vals[MAX_QTR_SENSORS], l_vals[MAX_QTR_SENSORS], r_vals[MAX_QTR_SENSORS];
    uint32_t t0 = HAL_GetTick();
    Mecanum_Chassis_t *c = (Mecanum_Chassis_t *)chassis;
    
    // reset calibration bounds
    if (front) {
        for (uint8_t i = 0; i < front->num_sensors; i++) { front->calibrated_minimums[i] = 4095; front->calibrated_maximums[i] = 0; }
    }
    if (left) {
        for (uint8_t i = 0; i < left->num_sensors; i++) { left->calibrated_minimums[i] = 4095; left->calibrated_maximums[i] = 0; }
    }
    if (right) {
        for (uint8_t i = 0; i < right->num_sensors; i++) { right->calibrated_minimums[i] = 4095; right->calibrated_maximums[i] = 0; }
    }
    

    
    while (HAL_GetTick() - t0 < duration_ms) {
        uint32_t elapsed = HAL_GetTick() - t0;
        if (c) {
            // Sequence requested: Rotate CW for 2s, Rotate CCW for 3s, then stop.
            if (elapsed < 2000) Chassis_Drive(c, 0, 0, -2000); // Rotate CW
            else if (elapsed < 5000) Chassis_Drive(c, 0, 0, 2000); // Rotate CCW
            else Chassis_Drive(c, 0, 0, 0); // Stop for the remaining 5s
        }
    
        if (poll_callback) poll_callback(); // Poll ADCs!
        
        if (front && front_adc) {
            QTR_ReadRaw(front, f_vals, front_adc);
            for (uint8_t i = 0; i < front->num_sensors; i++) {
                if (f_vals[i] > front->calibrated_maximums[i]) front->calibrated_maximums[i] = f_vals[i];
                if (f_vals[i] < front->calibrated_minimums[i]) front->calibrated_minimums[i] = f_vals[i];
            }
        }
        if (left && left_adc) {
            QTR_ReadRaw(left, l_vals, left_adc);
            for (uint8_t i = 0; i < left->num_sensors; i++) {
                if (l_vals[i] > left->calibrated_maximums[i]) left->calibrated_maximums[i] = l_vals[i];
                if (l_vals[i] < left->calibrated_minimums[i]) left->calibrated_minimums[i] = l_vals[i];
            }
        }
        if (right && right_adc) {
            QTR_ReadRaw(right, r_vals, right_adc);
            for (uint8_t i = 0; i < right->num_sensors; i++) {
                if (r_vals[i] > right->calibrated_maximums[i]) right->calibrated_maximums[i] = r_vals[i];
                if (r_vals[i] < right->calibrated_minimums[i]) right->calibrated_minimums[i] = r_vals[i];
            }
        }
        HAL_Delay(5);
    }
    
    if (c) Chassis_Drive(c, 0, 0, 0); // Stop after calibration
    
    if (c) Chassis_Drive(c, 0, 0, 0); // stop
    if (front) front->is_calibrated = 1;
    if (left) left->is_calibrated = 1;
    if (right) right->is_calibrated = 1;
    printf("Global Calibration Complete!\r\n");
}
