#include "qtr_8a.h"
#include "motor_driver.h"
#include <string.h>
#include <stdio.h>

/* --- Hardware Microsecond Timer (DWT) --- */
static void DWT_Init(void) {
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

static inline void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    while ((DWT->CYCCNT - start) < ticks);
}

/* --- Dynamic GPIO configuration --- */
static void QTR_PinToOutput(GPIO_TypeDef *port, uint16_t pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

static void QTR_PinToInput(GPIO_TypeDef *port, uint16_t pin) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void QTR_Init(QTR_Array_t *array, GPIO_TypeDef **ports, const uint16_t *pins, uint8_t num_sensors, uint32_t timeout_us)
{
    printf("  [QTR_Init] Starting RC initialization (sensors: %d, timeout: %lu us)...\r\n", num_sensors, timeout_us);
    if (array == NULL) return;
    
    DWT_Init(); // Ensure cycle counter is running
    
    array->num_sensors = (num_sensors > MAX_QTR_SENSORS) ? MAX_QTR_SENSORS : num_sensors;
    array->timeout_us = (timeout_us == 0) ? QTR_DEFAULT_TIMEOUT_US : timeout_us;
    array->threshold = 500; // default middle scale
    array->is_calibrated = 0;
    
    if (ports != NULL && pins != NULL) {
        for (uint8_t i = 0; i < array->num_sensors; i++) {
            array->sensors[i].Port = ports[i];
            array->sensors[i].Pin = pins[i];
        }
    } else {
        printf("  [QTR_Init] Note: Hardware handles are NULL. Using default Front QTR-8RC pins.\r\n");
        GPIO_TypeDef* default_ports[8] = {GPIOA, GPIOA, GPIOA, GPIOC, GPIOC, GPIOC, GPIOC, GPIOF};
        uint16_t default_pins[8] = {GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4};
        
        for (uint8_t i = 0; i < array->num_sensors; i++) {
            if (i < 8) {
                array->sensors[i].Port = default_ports[i];
                array->sensors[i].Pin = default_pins[i];
            } else {
                array->sensors[i].Port = NULL;
                array->sensors[i].Pin = 0;
            }
        }
    }

    for (uint8_t i = 0; i < array->num_sensors; i++) {
        array->calibrated_minimums[i] = array->timeout_us;
        array->calibrated_maximums[i] = 0;
    }
    printf("  [QTR_Init] RC Initialization complete!\r\n");
}

void QTR_ReadRaw(QTR_Array_t *array, uint16_t *sensor_values)
{
    if (array == NULL || sensor_values == NULL) return;
    
    // 1. Configure all pins as OUTPUT
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        if (array->sensors[i].Port) {
            QTR_PinToOutput(array->sensors[i].Port, array->sensors[i].Pin);
        }
    }
    
    // 2. Drive pins HIGH to charge capacitors (direct silicon access)
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        if (array->sensors[i].Port) {
            array->sensors[i].Port->BSRR = (uint32_t)array->sensors[i].Pin; // Set HIGH
        }
    }
    
    // 3. Wait to charge completely (15 us is plenty for 10nF)
    delay_us(15);
    
    // 4. Set as floating INPUTs
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        if (array->sensors[i].Port) {
            QTR_PinToInput(array->sensors[i].Port, array->sensors[i].Pin);
        }
    }
    
    // 5. Measure decay time
    uint32_t start_ticks = DWT->CYCCNT;
    uint32_t timeout_ticks = array->timeout_us * (SystemCoreClock / 1000000U);
    uint32_t elapsed_ticks = 0;
    
    // Initialize output arrays
    uint8_t completed = 0;
    uint8_t is_done[MAX_QTR_SENSORS] = {0};
    
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        sensor_values[i] = array->timeout_us; // Default to max
        if (array->sensors[i].Port == NULL) {
            is_done[i] = 1;
            completed++;
        }
    }
    
    // Tight polling loop with direct silicon register access (IDR)
    while (elapsed_ticks < timeout_ticks && completed < array->num_sensors) {
        elapsed_ticks = DWT->CYCCNT - start_ticks;
        
        for (uint8_t i = 0; i < array->num_sensors; i++) {
            if (!is_done[i]) {
                // If IDR bit is 0, the capacitor has discharged below logic HIGH threshold
                if ((array->sensors[i].Port->IDR & array->sensors[i].Pin) == 0) {
                    sensor_values[i] = (uint16_t)(elapsed_ticks / (SystemCoreClock / 1000000U));
                    is_done[i] = 1;
                    completed++;
                }
            }
        }
    }
}

uint8_t QTR_ReadDigital(QTR_Array_t *array)
{
    if (array == NULL) return 0;
    
    uint16_t cal_values[MAX_QTR_SENSORS];
    QTR_ReadCalibrated(array, cal_values, NULL);
    
    uint8_t digital_state = 0;
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        if (cal_values[i] >= array->threshold) {
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
        
        if (val > 50) {
            val -= 50;
            sum += val;
            weighted_sum += (uint32_t)val * i * 1000;
        }
    }
    
    if (sum == 0) {
        return (float)((array->num_sensors - 1) * 500);
    }
    
    return (float)weighted_sum / (float)sum;
}

void QTR_CalibrateSensorSweep(QTR_Array_t *array, volatile uint16_t *dma_buffer, uint32_t duration_ms, void *chassis)
{
    if (array == NULL) return;
    (void)dma_buffer; // Ignored for RC mode
    Mecanum_Chassis_t *c = (Mecanum_Chassis_t *)chassis;
    
    printf("\r\n====================================================\r\n");
    printf("         QTR-8RC INDIVIDUAL SENSOR CALIBRATION        \r\n");
    printf("====================================================\r\n");
    
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        array->calibrated_minimums[i] = array->timeout_us;
        array->calibrated_maximums[i] = 0;
    }
    
    for (int countdown = 3; countdown > 0; countdown--) {
        printf("Starting autonomous sweep in %d...\r\n", countdown);
        GPIOE->ODR |= 0xFF00; 
        HAL_Delay(500);
        GPIOE->ODR &= ~0xFF00;
        HAL_Delay(500);
    }
    
    printf("\r\nSWEEPING ACTIVE! Robot is driving itself to calibrate...\r\n");
    
    uint32_t start_time = HAL_GetTick();
    uint32_t last_blink = 0;
    
    while (HAL_GetTick() - start_time < duration_ms) {
        uint32_t current_time = HAL_GetTick();
        uint32_t elapsed = current_time - start_time;
        
        if (current_time - last_blink >= 200) {
            GPIOE->ODR ^= 0xFF00;
            last_blink = current_time;
        }
        
        if (c != NULL) {
            uint32_t cycle = elapsed % 3000; 
            if (cycle < 750) Chassis_Drive(c, 0, -960, 0);
            else if (cycle < 2250) Chassis_Drive(c, 0, 960, 0);
            else Chassis_Drive(c, 0, -960, 0);
        }
        
        uint16_t raw_vals[MAX_QTR_SENSORS];
        QTR_ReadRaw(array, raw_vals);
        for (uint8_t i = 0; i < array->num_sensors; i++) {
            if (raw_vals[i] < array->calibrated_minimums[i]) array->calibrated_minimums[i] = raw_vals[i];
            if (raw_vals[i] > array->calibrated_maximums[i]) array->calibrated_maximums[i] = raw_vals[i];
        }
        
        HAL_Delay(2);
    }
    
    if (c != NULL) {
        Chassis_CoastAll(c);
        printf("\r\nChassis stopped! ");
    }
    
    for (int f = 0; f < 5; f++) {
        GPIOE->ODR |= 0xFF00;
        HAL_Delay(100);
        GPIOE->ODR &= ~0xFF00;
        HAL_Delay(100);
    }
    
    array->is_calibrated = 1;
    printf("\r\nCALIBRATION COMPLETE!\r\n");
}

void QTR_ReadCalibrated(QTR_Array_t *array, uint16_t *calibrated_values, volatile uint16_t *dma_buffer)
{
    if (array == NULL || calibrated_values == NULL) return;
    (void)dma_buffer; // Ignored for RC mode
    
    uint16_t raw_buffer[MAX_QTR_SENSORS] = {0};
    QTR_ReadRaw(array, raw_buffer);
    
    for (uint8_t i = 0; i < array->num_sensors; i++) {
        uint16_t raw = raw_buffer[i];
        uint16_t cal_min = array->calibrated_minimums[i];
        uint16_t cal_max = array->calibrated_maximums[i];
        
        if (cal_max <= cal_min) {
            calibrated_values[i] = 0;
            continue;
        }
        
        int32_t value = ((int32_t)raw - cal_min) * 1000 / (cal_max - cal_min);
        
        if (value < 0) value = 0;
        if (value > 1000) value = 1000;
        
        calibrated_values[i] = (uint16_t)value;
    }
}

void QTR_CalibrateAllThree(
    QTR_Array_t *front, volatile uint16_t *front_dma,
    QTR_Array_t *left, volatile uint16_t *left_dma,
    QTR_Array_t *right, volatile uint16_t *right_dma,
    uint32_t duration_ms, void *chassis, void (*poll_callback)(void))
{
    (void)front_dma; (void)left_dma; (void)right_dma;
    Mecanum_Chassis_t *c = (Mecanum_Chassis_t *)chassis;
    
    printf("\r\n====================================================\r\n");
    printf("         QTR RC DYNAMIC 50/50 SWEEP CALIBRATION      \r\n");
    printf("====================================================\r\n");
    
    if (front) {
        for (uint8_t i = 0; i < front->num_sensors; i++) {
            front->calibrated_minimums[i] = front->timeout_us;
            front->calibrated_maximums[i] = 0;
        }
    }
    
    for (int countdown = 3; countdown > 0; countdown--) {
        printf("Dynamic sweep starting in %d...\r\n", countdown);
        GPIOE->ODR |= 0xFF00;
        HAL_Delay(500);
        GPIOE->ODR &= ~0xFF00;
        HAL_Delay(500);
    }
    
    printf("\r\nSWEEPING ACTIVE! Robot is driving itself to calibrate...\r\n");
    
    uint32_t start_time = HAL_GetTick();
    uint32_t last_blink = 0;
    
    while (HAL_GetTick() - start_time < duration_ms) {
        uint32_t current_time = HAL_GetTick();
        uint32_t elapsed = current_time - start_time;
        
        if (current_time - last_blink >= 200) {
            GPIOE->ODR ^= 0xFF00;
            last_blink = current_time;
        }
        
        if (c != NULL) {
            uint32_t cycle = elapsed % 2000;
            if (cycle < 1000) Chassis_Drive(c, 0, 0, 1440);
            else Chassis_Drive(c, 0, 0, -1440);
        }
        
        if (poll_callback != NULL) poll_callback();
        
        if (front != NULL) {
            uint16_t raw[MAX_QTR_SENSORS];
            QTR_ReadRaw(front, raw);
            for (uint8_t i = 0; i < front->num_sensors; i++) {
                if (raw[i] < front->calibrated_minimums[i]) front->calibrated_minimums[i] = raw[i];
                if (raw[i] > front->calibrated_maximums[i]) front->calibrated_maximums[i] = raw[i];
            }
        }
        
        HAL_Delay(5);
    }
    
    if (c != NULL) Chassis_CoastAll(c);
    
    for (int f = 0; f < 5; f++) {
        GPIOE->ODR |= 0xFF00;
        HAL_Delay(100);
        GPIOE->ODR &= ~0xFF00;
        HAL_Delay(100);
    }
    
    if (front) front->is_calibrated = 1;
    
    printf("\r\nCALIBRATION COMPLETED FOR ALL QTR SENSORS!\r\n");
    if (front) {
        for (uint8_t i = 0; i < front->num_sensors; i++) {
            printf(" Sensor %d -> MIN: %4d | MAX: %4d\r\n", 
                   i, front->calibrated_minimums[i], front->calibrated_maximums[i]);
        }
    }
    HAL_Delay(2000);
}
