#include "robot_core.h" /* must include robot_config.h */
#include "motor_driver.h"
#include "qtr_8a.h"
#include "pid.h"
#include "tcs34725.h"
#include "servo.h"
#include "encoders.h"
#include <stdio.h>

/* Distance Sensors */
#include "VL53L0X.h"
volatile uint16_t vl53_front_mm = 8190;
volatile uint16_t vl53_left_mm  = 8190;
volatile uint16_t vl53_right_mm = 8190;

static uint32_t last_loop_time = 0;
static uint32_t last_telemetry_time = 0;

/* External peripheral handles (defined in main.c) */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim6;

#if ENABLE_CHASSIS
static Mecanum_Chassis_t chassis;
#endif

static PID_t line_pid;

#if ENABLE_QTR_FRONT
static QTR_Array_t qtr_front;
static volatile uint16_t front_qtr_filtered[8] = {0};
#endif

#if ENABLE_QTR_RIGHT
static QTR_Array_t qtr_right;
static volatile uint16_t right_qtr_filtered[6] = {0};
#endif

#if ENABLE_QTR_LEFT
static QTR_Array_t qtr_left;
static volatile uint16_t left_qtr_filtered[6] = {0};
#endif

// Sharp sensors removed

#if ENABLE_COLOR_SENSOR
static TCS34725_RawData color_raw;
static DetectedColor color_result;
static uint32_t last_color_time = 0;
#endif

#if ENABLE_ARM_SERVO
static Elevator_t arm_elevator;
#endif

/* ---------------- ENCODERS ----------------
Bit order matches the tested main.c exactly:
enc1 = (PA9<<1)|PA10, enc2 = (PA8<<1)|PC9,
enc3 = (PC8<<1)|PC7, enc4 = (PC6<<1)|PD15
*/
#if ENABLE_ENCODERS
static const int8_t Q_TABLE[4][4] = {
    { 0, -1,  1,  0 },
    { 1,  0,  0, -1 },
    {-1,  0,  0,  1 },
    { 0,  1, -1,  0 }
};

static inline uint8_t rd_enc1(void) {
    return (uint8_t)((((GPIOA->IDR >> 9) & 1) << 1) | ((GPIOA->IDR >> 10) & 1));
}
static inline uint8_t rd_enc2(void) {
    return (uint8_t)((((GPIOA->IDR >> 8) & 1) << 1) | ((GPIOC->IDR >> 9) & 1));
}
static inline uint8_t rd_enc3(void) {
    return (uint8_t)((((GPIOC->IDR >> 8) & 1) << 1) | ((GPIOC->IDR >> 7) & 1));
}
static inline uint8_t rd_enc4(void) {
    return (uint8_t)((((GPIOC->IDR >> 6) & 1) << 1) | ((GPIOD->IDR >> 15) & 1));
}

static void Poll_Encoders(void)
{
    static uint8_t p1=0, p2=0, p3=0, p4=0, init=0;
    if (!init) { p1 = rd_enc1(); p2 = rd_enc2(); p3 = rd_enc3(); p4 = rd_enc4(); init=1; return; }

    uint8_t c1 = rd_enc1(); if(c1 != p1) { enc1_count += Q_TABLE[p1][c1]; p1 = c1; }
    uint8_t c2 = rd_enc2(); if(c2 != p2) { enc2_count -= Q_TABLE[p2][c2]; p2 = c2; }
    uint8_t c3 = rd_enc3(); if(c3 != p3) { enc3_count -= Q_TABLE[p3][c3]; p3 = c3; }
    uint8_t c4 = rd_enc4(); if(c4 != p4) { enc4_count += Q_TABLE[p4][c4]; p4 = c4; }
}
#endif

/* fires at 20 kHz from TIM6 */
static volatile uint8_t control_due = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) return;

#if ENABLE_ENCODERS
    Poll_Encoders(); /* every 50 us, never misses an edge */
#endif

    static uint16_t beat = 0;
    if (++beat >= 100) { /* 20000 / 100 = 200 Hz */
        beat = 0;
        control_due = 1;
    }
}



volatile uint16_t adc_front_buffer[8];
volatile uint16_t adc_right_buffer[6];
volatile uint16_t adc_left_buffer[6];

ADC_HandleTypeDef hadc4;

static uint16_t Read_ADC_Channel(ADC_HandleTypeDef *hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5; 
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    
    if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) return 4095;
    
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 2) == HAL_OK) {
        return HAL_ADC_GetValue(hadc);
    }
    return 4095;
}

static void Sensors_Poll(void)
{
#if ENABLE_QTR_FRONT
    adc_front_buffer[0] = Read_ADC_Channel(&hadc1, ADC_CHANNEL_2); // PA1
    adc_front_buffer[1] = Read_ADC_Channel(&hadc1, ADC_CHANNEL_3); // PA2
    adc_front_buffer[2] = Read_ADC_Channel(&hadc1, ADC_CHANNEL_4); // PA3
    adc_front_buffer[3] = Read_ADC_Channel(&hadc1, ADC_CHANNEL_5); // PF4
    adc_front_buffer[4] = Read_ADC_Channel(&hadc2, ADC_CHANNEL_1); // PA4
    adc_front_buffer[5] = Read_ADC_Channel(&hadc2, ADC_CHANNEL_2); // PA5
    adc_front_buffer[6] = Read_ADC_Channel(&hadc2, ADC_CHANNEL_3); // PA6
    adc_front_buffer[7] = Read_ADC_Channel(&hadc2, ADC_CHANNEL_4); // PA7
#endif

#if ENABLE_QTR_RIGHT
    adc_right_buffer[0] = Read_ADC_Channel(&hadc1, ADC_CHANNEL_6); // PC0
    adc_right_buffer[1] = Read_ADC_Channel(&hadc1, ADC_CHANNEL_7); // PC1
    adc_right_buffer[2] = Read_ADC_Channel(&hadc2, ADC_CHANNEL_11); // PC5
    adc_right_buffer[3] = Read_ADC_Channel(&hadc3, ADC_CHANNEL_12); // PB0
    adc_right_buffer[4] = Read_ADC_Channel(&hadc3, ADC_CHANNEL_1); // PB1
    adc_right_buffer[5] = Read_ADC_Channel(&hadc2, ADC_CHANNEL_12); // PB2
#endif

#if ENABLE_QTR_LEFT
    adc_left_buffer[0] = Read_ADC_Channel(&hadc1, ADC_CHANNEL_1); // PA0
    adc_left_buffer[1] = Read_ADC_Channel(&hadc3, ADC_CHANNEL_13); // PE7
    adc_left_buffer[2] = Read_ADC_Channel(&hadc1, ADC_CHANNEL_8); // PC2
    adc_left_buffer[3] = Read_ADC_Channel(&hadc1, ADC_CHANNEL_9); // PC3
    adc_left_buffer[4] = Read_ADC_Channel(&hadc4, ADC_CHANNEL_3); // PB12
    adc_left_buffer[5] = Read_ADC_Channel(&hadc4, ADC_CHANNEL_4); // PB13
#endif
}

/* ============================================================
INIT
Correct order:
1) hardware ADC self-calibration (BEFORE any conversion)
2) QTR_Init with NULL handles (no internal polling)
3) software sweep reading ADC polling only
4) start TIM6
============================================================ */

/* ============================================================
   ADC4 MANUAL INITIALIZATION (Fallback for CubeMX)
   ============================================================ */
static void ADC4_Manual_Init(void) {
    __HAL_RCC_ADC34_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    hadc4.Instance = ADC4;
    hadc4.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
    hadc4.Init.Resolution = ADC_RESOLUTION_12B;
    hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc4.Init.ContinuousConvMode = DISABLE;
    hadc4.Init.DiscontinuousConvMode = DISABLE;
    hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc4.Init.NbrOfConversion = 1;
    hadc4.Init.DMAContinuousRequests = DISABLE;
    hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc4.Init.LowPowerAutoWait = DISABLE;
    hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    HAL_ADC_Init(&hadc4);
}

void Robot_Init(void)
{
    ADC4_Manual_Init();
    /* Initialize PE8-PE15 as outputs for the LED bar/ring */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | 
                          GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    GPIOE->ODR &= ~0xFF00; // turn them all off initially
    printf("\033[2J\033[H");
    printf("======================================\r\n");
    printf("   NERC ROBOT CORE BOOTING            \r\n");
    printf("======================================\r\n");

#if ENABLE_CHASSIS
    chassis.fr.IN1_Port = GPIOD; chassis.fr.IN1_Pin = GPIO_PIN_2;
    chassis.fr.IN2_Port = GPIOD; chassis.fr.IN2_Pin = GPIO_PIN_1;
    chassis.fr.htim =&htim2; chassis.fr.channel = TIM_CHANNEL_1; chassis.fr.max_pwm=4800;

    chassis.rr.IN1_Port = GPIOD; chassis.rr.IN1_Pin = GPIO_PIN_0;
    chassis.rr.IN2_Port = GPIOC; chassis.rr.IN2_Pin = GPIO_PIN_12;
    chassis.rr.htim =&htim2; chassis.rr.channel = TIM_CHANNEL_2; chassis.rr.max_pwm=4800;

    chassis.rl.IN1_Port = GPIOB; chassis.rl.IN1_Pin = GPIO_PIN_14;
    chassis.rl.IN2_Port = GPIOB; chassis.rl.IN2_Pin = GPIO_PIN_15;
    chassis.rl.htim =&htim2; chassis.rl.channel = TIM_CHANNEL_3; chassis.rl.max_pwm=4800;

    chassis.fl.IN1_Port = GPIOD; chassis.fl.IN1_Pin = GPIO_PIN_8;
    chassis.fl.IN2_Port = GPIOD; chassis.fl.IN2_Pin = GPIO_PIN_9;
    chassis.fl.htim =&htim2; chassis.fl.channel = TIM_CHANNEL_4; chassis.fl.max_pwm=4800;

    chassis.STBY_Port = GPIOD; chassis.STBY_Pin = GPIO_PIN_5;
    Chassis_Init(&chassis);
#endif

    /* --- VL53L0X Sequential Initialization --- */
    // 1. Pull all XSHUT low to reset
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET); // FRONT
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); // LEFT
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // RIGHT
    HAL_Delay(10);
    
#if ENABLE_VL53L0X_FRONT
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET); // Boot FRONT
    HAL_Delay(10);
    VL53L0X_SelectSensor(0);
    setAddress(0x54);
    if (!initVL53L0X(1)) printf("VL53L0X FRONT Init Failed!\r\n");
    else { startContinuous(0); printf("VL53L0X FRONT Ready.\r\n"); }
#endif

#if ENABLE_VL53L0X_LEFT
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); // Boot LEFT
    HAL_Delay(10);
    VL53L0X_SelectSensor(1);
    setAddress(0x56);
    if (!initVL53L0X(1)) printf("VL53L0X LEFT Init Failed!\r\n");
    else { startContinuous(0); printf("VL53L0X LEFT Ready.\r\n"); }
#endif

#if ENABLE_VL53L0X_RIGHT
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET); // Boot RIGHT
    HAL_Delay(10);
    VL53L0X_SelectSensor(2);
    setAddress(0x58); // CHANGED: Avoids collision with TCS34725 (0x52)
    if (!initVL53L0X(1)) printf("VL53L0X RIGHT Init Failed!\r\n");
    else { startContinuous(0); printf("VL53L0X RIGHT Ready.\r\n"); }
#endif

    /* 3) QTR_Init for QTR-8A analog arrays */
#if ENABLE_QTR_FRONT
    QTR_Init(&qtr_front, 8);
    PID_Init(&line_pid, 1.60f, 0.0f, 0.00f, 500.0f, 1200.0f);
#endif

#if ENABLE_QTR_RIGHT
    QTR_Init(&qtr_right, 6);
#endif
#if ENABLE_QTR_LEFT
    QTR_Init(&qtr_left, 6);
#endif

    // Perform static global calibration for any initialized arrays
    /* -- CALIBRATION DISABLED FOR MOTOR TESTING --
    QTR_CalibrateAllThree(
#if ENABLE_QTR_FRONT
        &qtr_front, adc_front_buffer,
#else
        NULL, NULL,
#endif
#if ENABLE_QTR_LEFT
        &qtr_left, adc_left_buffer,
#else
        NULL, NULL,
#endif
#if ENABLE_QTR_RIGHT
        &qtr_right, adc_right_buffer,
#else
        NULL, NULL,
#endif
        5000, 
#if ENABLE_CHASSIS
        &chassis,
#else
        NULL,
#endif
        Sensors_Poll
    );
    */


#if ENABLE_COLOR_SENSOR
    if (TCS34725_Init(&hi2c1) == HAL_OK) printf("color sensor ready\r\n");
    else printf("color sensor FAILED\r\n");
    last_color_time = HAL_GetTick();
#endif

#if ENABLE_ARM_SERVO
    Elevator_Init(&arm_elevator, &htim3, TIM_CHANNEL_1);
#endif

    /* start TIM6 for encoders and 200 Hz control beat */
    HAL_TIM_Base_Start_IT(&htim6);

    last_loop_time = HAL_GetTick();
    last_telemetry_time = last_loop_time;
}

/* ============================================================
   MAIN LOOP (200 Hz Beat)
   ============================================================ */
void Robot_RunLoop(void)
{
    if (!control_due) return; /* wait for the 200 Hz beat */
    control_due = 0;
    const float dt = 0.005f; /* fixed 200 Hz step */

    int32_t omega = 0;
    int32_t base_speed = 2700;
    uint8_t force_brake = 0;

    /* Manually poll all ADCs for this frame */
    Sensors_Poll();

    /* --- VL53L0X Polling --- */
#if ENABLE_VL53L0X_FRONT
    VL53L0X_SelectSensor(0);
    vl53_front_mm = readRangeContinuousMillimeters(NULL);
#endif
#if ENABLE_VL53L0X_LEFT
    VL53L0X_SelectSensor(1);
    vl53_left_mm = readRangeContinuousMillimeters(NULL);
#endif
#if ENABLE_VL53L0X_RIGHT
    VL53L0X_SelectSensor(2);
    vl53_right_mm = readRangeContinuousMillimeters(NULL);
#endif

#if ENABLE_QTR_FRONT
    /* read digital RC time, filter, all at 200 Hz */
    uint16_t cal[8];
    QTR_ReadCalibrated(&qtr_front, cal, adc_front_buffer);
    for (int i = 0; i < 8; i++)
        front_qtr_filtered[i] =
            (uint16_t)((EMA_ALPHA * cal[i]) + ((1.0f - EMA_ALPHA) * front_qtr_filtered[i]));
// Sharp FRONT removed

    float position = QTR_GetLinePosition(&qtr_front, (uint16_t*)front_qtr_filtered);
    float error = LINE_CENTER - position;
    if (error < -LINE_DEADBAND || error > LINE_DEADBAND)
        omega = (int32_t)PID_Update(&line_pid, error, dt);
    else
        PID_Reset(&line_pid);

    uint8_t all_white = 1;
    uint16_t led_mask = 0;
    for (int i = 0; i < 8; i++) {
        if (front_qtr_filtered[i] >= 300) { 
            all_white = 0; 
            led_mask |= (1 << (8 + i)); // Light up PE8 + i
        }
    }
    if (all_white) force_brake = 1;
    
    // LED output moved to motor test
#endif

#if ENABLE_QTR_RIGHT
    {
    uint16_t right_cal[6];
    QTR_ReadCalibrated(&qtr_right, right_cal, adc_right_buffer);
    for (int i = 0; i < 6; i++)
        right_qtr_filtered[i] =
            (uint16_t)((EMA_ALPHA * right_cal[i]) + ((1.0f - EMA_ALPHA) * right_qtr_filtered[i]));
        
        // 6 sensors, max is 5000, center is 2500
        float position = QTR_GetLinePosition(&qtr_right, (uint16_t*)right_qtr_filtered);
        float error = 2500.0f - position; 
        
        if (error < -LINE_DEADBAND || error > LINE_DEADBAND)
            omega = (int32_t)PID_Update(&line_pid, error, dt);
        else
            PID_Reset(&line_pid);
            
        uint8_t all_white = 1;
        uint16_t led_mask = 0;
        
        // PURE HARDWARE DEBUG: Map the RAW analog voltage directly to the LEDs!
        // A value of 1000 means ~0.8V. If it's over black, it should be > 2000 (1.6V+).
        for (int i = 0; i < 6; i++) {
            if (right_qtr_filtered[i] >= 300) {
                all_white = 0;
            }
        }
        
        if (all_white) force_brake = 1;
        
        // LED output moved to motor test
    }
#endif

#if ENABLE_QTR_LEFT
    {
        uint16_t left_cal[6];
        QTR_ReadCalibrated(&qtr_left, left_cal, adc_left_buffer);
        for (int i = 0; i < 6; i++)
            left_qtr_filtered[i] =
                (uint16_t)((EMA_ALPHA * left_cal[i]) + ((1.0f - EMA_ALPHA) * left_qtr_filtered[i]));
// Sharp SIDE removed
    }
#endif

#if ENABLE_CHASSIS
    // --- MOTOR & LED SEQUENCE TEST ---
    // Forward -> Backward -> Right -> Left -> Spin -> Loop
    uint32_t t = HAL_GetTick() % 10000; // 10 second sequence
    uint16_t led_test_mask = 0;
    
    // Telemetry Printing (State tracking to only print once per change)
    static uint32_t last_test_state = 99;
    uint32_t current_state = t / 2000;
    
    if (current_state != last_test_state) {
        last_test_state = current_state;
        printf("\r\n=======================\r\n");
        if (current_state == 0) printf(">>> MOVING FORWARD <<<\r\n");
        else if (current_state == 1) printf(">>> MOVING BACKWARD <<<\r\n");
        else if (current_state == 2) printf(">>> STRAFING RIGHT <<<\r\n");
        else if (current_state == 3) printf(">>> STRAFING LEFT <<<\r\n");
        else if (current_state == 4) printf(">>> SPINNING IN PLACE <<<\r\n");
        printf("=======================\r\n");
    }
    
    if (t < 2000) {
        Chassis_Drive(&chassis, 2000, 0, 0); // Forward
        led_test_mask = (1 << 8) | (1 << 9); // PE8, PE9
    } else if (t < 4000) {
        Chassis_Drive(&chassis, -2000, 0, 0); // Backward
        led_test_mask = (1 << 10) | (1 << 11); // PE10, PE11
    } else if (t < 6000) {
        Chassis_Drive(&chassis, 0, 2000, 0); // Strafe Right
        led_test_mask = (1 << 12) | (1 << 13); // PE12, PE13
    } else if (t < 8000) {
        Chassis_Drive(&chassis, 0, -2000, 0); // Strafe Left
        led_test_mask = (1 << 14) | (1 << 15); // PE14, PE15
    } else {
        Chassis_Drive(&chassis, 0, 0, 2000); // Spin in place
        led_test_mask = 0xFF00; // ALL LEDs
    }
    
    // Output directly to LED pins PE8 to PE15
    GPIOE->ODR = (GPIOE->ODR & ~0xFF00) | led_test_mask;

    /* PURE LINE FOLLOWING BACKUP (Uncomment this and delete the sequence test when ready)
#if ENABLE_QTR_FRONT
    if (force_brake) Chassis_Drive(&chassis, 0, 0, 0);
    else Chassis_Drive(&chassis, -base_speed, 0, -omega);
#elif ENABLE_QTR_RIGHT
    if (force_brake) Chassis_Drive(&chassis, 0, 0, 0);
    else Chassis_Drive(&chassis, 0, -base_speed, omega); 
#else
    Chassis_Drive(&chassis, 0, 0, 0);
#endif
    */
#endif

#if ENABLE_TELEMETRY
    /* bench only. blocking UART. keep OFF for a scored run. */
    static uint32_t last_tele = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tele >= 100) {
        printf("\033[H");
        printf("==============================\r\n");
        printf("       NERC DASHBOARD         \r\n");
        printf("==============================\r\n");
#if ENABLE_QTR_FRONT
        printf("POS   %6.1f   OMEGA %6ld\r\n", (double)position, (long)omega);
        printf("CALIB [");
        for (int i = 0; i < 8; i++) printf("%4d ", front_qtr_filtered[i]);
        printf("]\r\n");
        printf("LINE  [");
        for (int i = 0; i < 8; i++)
            printf(front_qtr_filtered[i] >= 500 ? "#### " :
                  (front_qtr_filtered[i] >= 200 ? "---- " : "     "));
        printf("]\r\n");
#endif

        // Print distance sensors
        printf("Dist (mm) | ");
#if ENABLE_VL53L0X_FRONT
        printf("F: %4d ", vl53_front_mm);
#endif
#if ENABLE_VL53L0X_LEFT
        printf("L: %4d ", vl53_left_mm);
#endif
#if ENABLE_VL53L0X_RIGHT
        printf("R: %4d ", vl53_right_mm);
#endif
        printf("\r\n");
        printf("==============================\r\n");
        last_tele = now;
    }
#endif
}

/* ============================================================
   TURN ROUTINES
   ============================================================ */

#if ENABLE_CHASSIS

#define TURN_SPEED_FAST 800
#define TURN_SPEED_SLOW 400
#define LINE_ON 500 /* calibrated value: clearly on the line */
#define LINE_OFF 200 /* calibrated value: clearly off the line */
#define TURN_TIMEOUT_MS 3000

#if ENABLE_QTR_FRONT
/* strongest of the two center sensors, from a fresh raw frame */
static uint16_t Front_CenterStrength(void)
{
    uint16_t cal[8];
    QTR_ReadCalibrated(&qtr_front, cal, adc_front_buffer);
    return (cal[3] > cal[4]) ? cal[3] : cal[4];
}

/* dir = +1 clockwise (right), -1 counter-clockwise (left). */
void Turn90_LineSnap(int8_t dir)
{
    uint32_t t0 = HAL_GetTick();

    /* phase 1: leave the current line at speed */
    Chassis_Drive(&chassis, 0, 0, dir * TURN_SPEED_FAST);
    while (Front_CenterStrength() > LINE_OFF) {
        if (HAL_GetTick() - t0 > TURN_TIMEOUT_MS) break;
    }

    /* phase 2: creep until the next perpendicular line is centered */
    Chassis_Drive(&chassis, 0, 0, dir * TURN_SPEED_SLOW);
    while (Front_CenterStrength() < LINE_ON) {
        if (HAL_GetTick() - t0 > TURN_TIMEOUT_MS) break;
    }

    Chassis_Drive(&chassis, 0, 0, 0); /* coast, no hard brake */
}
#endif // ENABLE_QTR_FRONT

#if ENABLE_ENCODERS
/* rough tick rotate for 45s and any turn with no line to snap to.
   ticks is the |front-left encoder| change for the angle, tuned once. */
void Turn_ByTicks(int8_t dir, int32_t ticks)
{
    Encoders_ResetAll();
    uint32_t t0 = HAL_GetTick();
    Chassis_Drive(&chassis, 0, 0, dir * TURN_SPEED_SLOW);
    while (1) {
        int32_t e = enc4_count; if (e < 0) e = -e;
        if (e >= ticks) break;
        if (HAL_GetTick() - t0 > TURN_TIMEOUT_MS) break;
    }
    Chassis_Drive(&chassis, 0, 0, 0);
}
#endif // ENABLE_ENCODERS

#endif /* ENABLE_CHASSIS */
