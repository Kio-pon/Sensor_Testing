#include "qtr_8a.h"
#include "motor_driver.h"
#include <string.h>
#include <stdio.h>

void QTR_Init(QTR_Array_t *array, ADC_HandleTypeDef **hadc_list, const uint32_t *channels, uint8_t num_sensors, uint16_t threshold)
{
    printf("  [QTR_Init] Starting initialization (sensors: %d, threshold: %d)...\r\n", num_sensors, threshold);
    if (array == NULL || hadc_list == NULL || channels == NULL) {
        printf("  [QTR_Init] ERROR: Null parameter(s) passed!\r\n");
        return;
    }
    
    array->num_sensors = (num_sensors > MAX_QTR_SENSORS) ? MAX_QTR_SENSORS : num_sensors;
    array->threshold = (threshold == 0) ? QTR_DEFAULT_THRESHOLD : threshold;
    
    ADC_HandleTypeDef *calibrated_adcs[MAX_QTR_SENSORS] = {NULL};
    uint8_t num_calibrated = 0;

    // Copy the channel and ADC mappings
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        array->sensors[i].hadc = hadc_list[i];
        array->sensors[i].channel = channels[i];
        
        // Trigger self-calibration for this ADC instance if not already done
        if (hadc_list[i] != NULL) {
            uint8_t already_calibrated = 0;
            for (uint8_t j = 0; j < num_calibrated; j++) {
                if (calibrated_adcs[j] == hadc_list[i]) {
                    already_calibrated = 1;
                    break;
                }
            }
            if (!already_calibrated) {
                printf("  [QTR_Init] Triggering self-calibration for ADC handle %p...\r\n", (void*)hadc_list[i]);
                HAL_StatusTypeDef cal_status = HAL_ADCEx_Calibration_Start(hadc_list[i], ADC_SINGLE_ENDED);
                printf("  [QTR_Init] Self-calibration for ADC handle %p completed with status: %d\r\n", (void*)hadc_list[i], (int)cal_status);
                calibrated_adcs[num_calibrated++] = hadc_list[i];
            }
        }
    }
    // Set default calibration values
    array->is_calibrated = 0;
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        array->calibrated_minimums[i] = 0;      // default light min
        array->calibrated_maximums[i] = 4095;   // default dark max
    }
    printf("  [QTR_Init] Initialization complete!\r\n");
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

void QTR_CalibrateSensorSweep(QTR_Array_t *array, volatile uint16_t *dma_buffer, uint32_t duration_ms, void *chassis)
{
    if (array == NULL || dma_buffer == NULL) return;
    Mecanum_Chassis_t *c = (Mecanum_Chassis_t *)chassis;
    
    printf("\r\n====================================================\r\n");
    printf("         QTR-8A INDIVIDUAL SENSOR CALIBRATION        \r\n");
    printf("====================================================\r\n");
    printf("INSTRUCTIONS:\r\n");
    printf("1. Put the robot roughly centered over the line.\r\n");
    printf("2. The robot will automatically strafe left & right\r\n");
    printf("   to sweep all sensors over the line to calibrate.\r\n");
    printf("----------------------------------------------------\r\n");
    
    // Initialize per-sensor bounds individually
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        array->calibrated_minimums[i] = 4095;
        array->calibrated_maximums[i] = 0;
    }
    
    // Countdown with clear visual LED blinks
    for (int countdown = 3; countdown > 0; countdown--) {
        printf("Starting autonomous sweep in %d...\r\n", countdown);
        
        // Blink PE8 to PE15 to visually announce countdown
        GPIOE->ODR |= 0xFF00; // Turn on all 8 Discovery LEDs
        HAL_Delay(500);
        GPIOE->ODR &= ~0xFF00; // Turn off all 8 Discovery LEDs
        HAL_Delay(500);
    }
    
    printf("\r\nSWEEPING ACTIVE! Robot is driving itself to calibrate...\r\n");
    
    uint32_t start_time = HAL_GetTick();
    uint32_t last_tick = 0;
    uint32_t last_blink = 0;
    
    while (HAL_GetTick() - start_time < duration_ms) {
        uint32_t current_time = HAL_GetTick();
        uint32_t elapsed = current_time - start_time;
        
        // Print progress dot and live raw S0 channel reading every 500ms
        if (current_time - last_tick >= 500) {
            printf("[%d] ", (int)dma_buffer[0]);
            fflush(stdout);
            last_tick = current_time;
        }
        
        // Toggle/Blink all 8 LEDs every 200ms during active sweep for clear status
        if (current_time - last_blink >= 200) {
            GPIOE->ODR ^= 0xFF00; // Toggle PE8 to PE15
            last_blink = current_time;
        }
        
        // Active strafing motion: strafe left for some time, then right, etc.
        // We sweep left and right (sideways sliding) at exactly 20% speed (960 compare value out of 4800 max)
        if (c != NULL) {
            uint32_t cycle = elapsed % 3000; // 3-second full cycle
            if (cycle < 750) {
                // Strafe Left at 20% speed command
                Chassis_Drive(c, 0, -960, 0);
            } else if (cycle < 2250) {
                // Strafe Right at 20% speed command
                Chassis_Drive(c, 0, 960, 0);
            } else {
                // Strafe Left at 20% speed command
                Chassis_Drive(c, 0, -960, 0);
            }
        }
        
        // Sample continuous DMA buffer and update per-sensor bounds individually
        for (uint8_t i = 0; i < array->num_sensors; i++) {
            uint16_t val = dma_buffer[i];
            if (val > 200) { // filter out zero-readings and low noise values
                if (val < array->calibrated_minimums[i]) {
                    array->calibrated_minimums[i] = val;
                }
                if (val > array->calibrated_maximums[i]) {
                    array->calibrated_maximums[i] = val;
                }
            }
        }
        
        // Sleep very briefly to avoid tight CPU loop
        HAL_Delay(2);
    }
    
    // Stop the chassis once calibration completes
    if (c != NULL) {
        Chassis_CoastAll(c);
        printf("\r\nChassis stopped! ");
    }
    
    // Rapidly flash all 8 LEDs 5 times to confirm calibration completed!
    printf("\r\nFLASHING STATUS LEDS ON DISCOVERY BOARD...\r\n");
    for (int f = 0; f < 5; f++) {
        GPIOE->ODR |= 0xFF00;  // All ON
        HAL_Delay(100);
        GPIOE->ODR &= ~0xFF00; // All OFF
        HAL_Delay(100);
    }
    
    // Safety check in case calibration bounds were never updated (e.g. all 0)
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        if (array->calibrated_minimums[i] >= array->calibrated_maximums[i]) {
            // Revert to default light/dark limits if invalid
            array->calibrated_minimums[i] = 1000;
            array->calibrated_maximums[i] = 3000;
        }
    }
    
    array->is_calibrated = 1;
    printf("\r\nCALIBRATION COMPLETE!\r\n");
    printf("====================================================\r\n");
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        printf(" Sensor %d -> MIN (White): %4d | MAX (Black): %4d | Range: %4d\r\n", 
               i, array->calibrated_minimums[i], array->calibrated_maximums[i], 
               array->calibrated_maximums[i] - array->calibrated_minimums[i]);
    }
    printf("====================================================\r\n");
    HAL_Delay(3000); // Let user read summary on PuTTY
}

void QTR_ReadCalibrated(QTR_Array_t *array, uint16_t *calibrated_values, volatile uint16_t *dma_buffer)
{
    if (array == NULL || calibrated_values == NULL) return;
    
    uint16_t raw_buffer[MAX_QTR_SENSORS] = {0};
    if (dma_buffer == NULL) {
        QTR_ReadRaw(array, raw_buffer);
    }
    
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        uint16_t raw = (dma_buffer != NULL) ? dma_buffer[i] : raw_buffer[i];
        uint16_t cal_min = array->calibrated_minimums[i];
        uint16_t cal_max = array->calibrated_maximums[i];
        
        if (cal_max <= cal_min) {
            // Avoid division by zero, default to raw scale
            calibrated_values[i] = raw;
            continue;
        }
        
        int32_t value = ((int32_t)raw - cal_min) * 1000 / (cal_max - cal_min);
        
        if (value < 0) value = 0;
        if (value > 1000) value = 1000;
        
        calibrated_values[i] = (uint16_t)value;
    }
}

// Unified multi-sensor sweep calibration at exactly 20% speed (960 compare)
void QTR_CalibrateAllThree(QTR_Array_t *front, QTR_Array_t *left, QTR_Array_t *right, volatile uint16_t *dma_buffer, uint32_t duration_ms, void *chassis)
{
    if (front == NULL || left == NULL || right == NULL || dma_buffer == NULL) return;
    Mecanum_Chassis_t *c = (Mecanum_Chassis_t *)chassis;
    
    printf("\r\n====================================================\r\n");
    printf("         QTR ALL-3-SENSOR SWEEP CALIBRATION          \r\n");
    printf("====================================================\r\n");
    printf("Robot will strafe left/right at 20%% speed for 10s.\r\n");
    
    // Initialize bounds for all three arrays
    for (uint8_t i = 0; i < front->num_sensors; i++) {
        front->calibrated_minimums[i] = 4095;
        front->calibrated_maximums[i] = 0;
    }
    for (uint8_t i = 0; i < left->num_sensors; i++) {
        left->calibrated_minimums[i] = 4095;
        left->calibrated_maximums[i] = 0;
    }
    for (uint8_t i = 0; i < right->num_sensors; i++) {
        right->calibrated_minimums[i] = 4095;
        right->calibrated_maximums[i] = 0;
    }
    
    // Countdown blinks
    for (int countdown = 3; countdown > 0; countdown--) {
        printf("Sweep starting in %d...\r\n", countdown);
        GPIOE->ODR |= 0xFF00; // Turn on LEDs
        HAL_Delay(500);
        GPIOE->ODR &= ~0xFF00; // Turn off LEDs
        HAL_Delay(500);
    }
    
    printf("\r\nSWEEPING ACTIVE! Calibrating FRONT, LEFT, & RIGHT QTRs...\r\n");
    
    uint32_t start_time = HAL_GetTick();
    uint32_t last_blink = 0;
    uint32_t last_print = 0;
    
    while (HAL_GetTick() - start_time < duration_ms) {
        uint32_t current_time = HAL_GetTick();
        uint32_t elapsed = current_time - start_time;
        
        // Blink PE8 to PE15 during calibration
        if (current_time - last_blink >= 200) {
            GPIOE->ODR ^= 0xFF00;
            last_blink = current_time;
        }
        
        // Print progress
        if (current_time - last_print >= 1000) {
            printf("Calibrating... %d%%\r\n", (int)(elapsed * 100 / duration_ms));
            last_print = current_time;
        }
        
        // Strafe Left/Right at 20% speed (960 compare value out of 4800)
        if (c != NULL) {
            uint32_t cycle = elapsed % 3000;
            if (cycle < 750) {
                Chassis_Drive(c, 0, -960, 0); // Strafe Left
            } else if (cycle < 2250) {
                Chassis_Drive(c, 0, 960, 0);  // Strafe Right
            } else {
                Chassis_Drive(c, 0, -960, 0); // Strafe Left
            }
        }
        
        // 1. Update front array (using DMA buffer)
        for (uint8_t i = 0; i < front->num_sensors; i++) {
            uint16_t val = dma_buffer[i];
            if (val > 200) {
                if (val < front->calibrated_minimums[i]) front->calibrated_minimums[i] = val;
                if (val > front->calibrated_maximums[i]) front->calibrated_maximums[i] = val;
            }
        }
        
        // 2. Update left array (standard analog read)
        uint16_t left_raw[MAX_QTR_SENSORS] = {0};
        QTR_ReadRaw(left, left_raw);
        for (uint8_t i = 0; i < left->num_sensors; i++) {
            uint16_t val = left_raw[i];
            if (val > 200) {
                if (val < left->calibrated_minimums[i]) left->calibrated_minimums[i] = val;
                if (val > left->calibrated_maximums[i]) left->calibrated_maximums[i] = val;
            }
        }
        
        // 3. Update right array (standard analog read)
        uint16_t right_raw[MAX_QTR_SENSORS] = {0};
        QTR_ReadRaw(right, right_raw);
        for (uint8_t i = 0; i < right->num_sensors; i++) {
            uint16_t val = right_raw[i];
            if (val > 200) {
                if (val < right->calibrated_minimums[i]) right->calibrated_minimums[i] = val;
                if (val > right->calibrated_maximums[i]) right->calibrated_maximums[i] = val;
            }
        }
        
        HAL_Delay(5);
    }
    
    // Stop the chassis
    if (c != NULL) {
        Chassis_CoastAll(c);
        printf("\r\nChassis stopped! ");
    }
    
    // Rapidly flash all 8 LEDs 5 times to confirm calibration completed!
    printf("\r\nFLASHING STATUS LEDS ON DISCOVERY BOARD...\r\n");
    for (int f = 0; f < 5; f++) {
        GPIOE->ODR |= 0xFF00;  // All ON
        HAL_Delay(100);
        GPIOE->ODR &= ~0xFF00; // All OFF
        HAL_Delay(100);
    }
    
    // Final check for valid calibration limits
    for (uint8_t i = 0; i < front->num_sensors; i++) {
        if (front->calibrated_minimums[i] >= front->calibrated_maximums[i]) {
            front->calibrated_minimums[i] = 1000;
            front->calibrated_maximums[i] = 3000;
        }
    }
    for (uint8_t i = 0; i < left->num_sensors; i++) {
        if (left->calibrated_minimums[i] >= left->calibrated_maximums[i]) {
            left->calibrated_minimums[i] = 1000;
            left->calibrated_maximums[i] = 3000;
        }
    }
    for (uint8_t i = 0; i < right->num_sensors; i++) {
        if (right->calibrated_minimums[i] >= right->calibrated_maximums[i]) {
            right->calibrated_minimums[i] = 1000;
            right->calibrated_maximums[i] = 3000;
        }
    }
    
    front->is_calibrated = 1;
    left->is_calibrated = 1;
    right->is_calibrated = 1;
    
    printf("\r\nCALIBRATION COMPLETED FOR ALL 3 QTR SENSORS!\r\n");
    printf("====================================================\r\n");
    HAL_Delay(2000);
}

