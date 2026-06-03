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
extern ADC_HandleTypeDef hadc4;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim6;

#if ENABLE_CHASSIS
static Mecanum_Chassis_t chassis;
#endif

static PID_t line_pid_front;
static PID_t line_pid_right;

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


static uint16_t Read_ADC_Channel(ADC_HandleTypeDef *hadc, uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5; 
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
    adc_left_buffer[5] = Read_ADC_Channel(&hadc3, ADC_CHANNEL_5); // PB13 (ADC3_IN5)
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

void Robot_Init(void)
{
    // ADC4 is now properly initialized by STM32CubeMX in main.c!
    
#if ENABLE_LED_BAR
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
#endif

    printf("\033[2J\033[H");
    printf("======================================\r\n");
    printf("        Antigravity IDE - INIT        \r\n");
    printf("======================================\r\n");

    // Force ALL sensor pins to ANALOG mode.
    // This is REQUIRED because MX_GPIO_Init() runs before the ADC inits 
    // and can leave some pins configured as digital, causing ADC reads of 4095.
    GPIO_InitTypeDef GPIO_InitStruct_QTR = {0};
    GPIO_InitStruct_QTR.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct_QTR.Pull = GPIO_NOPULL;

    // Front array: PA1-PA7, PF4
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct_QTR.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_QTR);

    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitStruct_QTR.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct_QTR);

    // Right array: PC0, PC1, PC5, PB0, PB1, PB2
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct_QTR.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct_QTR);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct_QTR.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct_QTR);

    // Calibrate all ADCs to ensure they work properly on STM32F3!
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);

    // FIX: The Left QTR array might be physically powered off by the XSHUT_LEFT_Pin!
    // We will forcibly pull it HIGH to turn the IR emitters back on.
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); // PD11 is XSHUT_LEFT_Pin

#if ENABLE_CHASSIS
    // Reversed FR
    chassis.fr.IN1_Port = GPIOD; chassis.fr.IN1_Pin = GPIO_PIN_1;
    chassis.fr.IN2_Port = GPIOD; chassis.fr.IN2_Pin = GPIO_PIN_2;
    chassis.fr.htim =&htim2; chassis.fr.channel = TIM_CHANNEL_1; chassis.fr.max_pwm=4800;

    // Reversed RR
    chassis.rr.IN1_Port = GPIOC; chassis.rr.IN1_Pin = GPIO_PIN_12;
    chassis.rr.IN2_Port = GPIOD; chassis.rr.IN2_Pin = GPIO_PIN_0;
    chassis.rr.htim =&htim2; chassis.rr.channel = TIM_CHANNEL_2; chassis.rr.max_pwm=4800;

    chassis.rl.IN1_Port = GPIOB; chassis.rl.IN1_Pin = GPIO_PIN_4; // Changed from PB14
    chassis.rl.IN2_Port = GPIOB; chassis.rl.IN2_Pin = GPIO_PIN_5; // Changed from PB15
    chassis.rl.htim =&htim2; chassis.rl.channel = TIM_CHANNEL_3; chassis.rl.max_pwm=4800;

    // Reversed FL
    chassis.fl.IN1_Port = GPIOD; chassis.fl.IN1_Pin = GPIO_PIN_9;
    chassis.fl.IN2_Port = GPIOD; chassis.fl.IN2_Pin = GPIO_PIN_8;
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
    PID_Init(&line_pid_front, 0.80f, 0.0f, 0.00f, 500.0f, 1200.0f);
    PID_Init(&line_pid_right, 0.80f, 0.0f, 0.00f, 500.0f, 1200.0f);
#endif

#if ENABLE_QTR_RIGHT
    QTR_Init(&qtr_right, 6);
#endif
#if ENABLE_QTR_LEFT
    QTR_Init(&qtr_left, 6);
#endif

    // Startup Calibration Sequence (Removed as requested)
    printf("Global Calibration Skipped. Using hardcoded 3950 threshold.\r\n");

/* #if ENABLE_COLOR_SENSOR
    if (TCS34725_Init(&hi2c1) == HAL_OK) printf("color sensor ready\r\n");
    else printf("color sensor FAILED\r\n");
    last_color_time = HAL_GetTick();
#endif */

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

    int32_t omega_front = 0;
    int32_t omega_right = 0;
    int32_t base_speed = 2400; // 50% of 4800 (max PWM)
    uint8_t front_black_count = 0;
    uint8_t right_black_count = 0;

    /* Manually poll all ADCs for this frame */
    Sensors_Poll();

    /* ---- FRONT ARRAY PROCESSING ---- */
#if ENABLE_QTR_FRONT
    {
        uint16_t cal[8];
        QTR_ReadCalibrated(&qtr_front, cal, adc_front_buffer);
        for (int i = 0; i < 8; i++)
            front_qtr_filtered[i] =
                (uint16_t)((EMA_ALPHA * cal[i]) + ((1.0f - EMA_ALPHA) * front_qtr_filtered[i]));

        /* Count how many front sensors see black */
        for (int i = 0; i < 8; i++) {
            if (front_qtr_filtered[i] >= 400) front_black_count++;
        }

        /* PID only when at least 2 middle sensors see line (robust) */
        uint8_t middle_black = 0;
        for (int i = 2; i < 6; i++) { // sensors 2,3,4,5 are "middle"
            if (front_qtr_filtered[i] >= 400) middle_black++;
        }

        if (middle_black >= 2) {
            float position = QTR_GetLinePosition(&qtr_front, (uint16_t*)front_qtr_filtered);
            float error = LINE_CENTER - position;
            if (error < -LINE_DEADBAND || error > LINE_DEADBAND)
                omega_front = (int32_t)PID_Update(&line_pid_front, error, dt);
            else
                PID_Reset(&line_pid_front);
        } else {
            PID_Reset(&line_pid_front);
        }
    }
#endif

    /* ---- RIGHT ARRAY PROCESSING ---- */
#if ENABLE_QTR_RIGHT
    {
        uint16_t right_cal[6];
        QTR_ReadCalibrated(&qtr_right, right_cal, adc_right_buffer);
        for (int i = 0; i < 6; i++)
            right_qtr_filtered[i] =
                (uint16_t)((EMA_ALPHA * right_cal[i]) + ((1.0f - EMA_ALPHA) * right_qtr_filtered[i]));

        /* Count how many right sensors see black */
        for (int i = 0; i < 6; i++) {
            if (right_qtr_filtered[i] >= 400) right_black_count++;
        }

        /* PID for right array (6 sensors, center = 2500) */
        float position_r = QTR_GetLinePosition(&qtr_right, (uint16_t*)right_qtr_filtered);
        float error_r = 2500.0f - position_r;
        if (error_r < -LINE_DEADBAND || error_r > LINE_DEADBAND)
            omega_right = (int32_t)PID_Update(&line_pid_right, error_r, dt);
        else
            PID_Reset(&line_pid_right);
    }
#endif

    /* ---- STATE MACHINE ---- */
#if ENABLE_CHASSIS
    static uint8_t robot_state = 0;
    static uint32_t state_start = 0;

    if (robot_state == 0) {
        /* ====== STATE 0: LINE FOLLOWING (FRONT) ====== */
        static uint32_t state0_start = 0;
        if (state0_start == 0) state0_start = HAL_GetTick();

        if (front_black_count == 0) {
            Chassis_Drive(&chassis, 0, 0, 0); // Stop if we fall off
        } else {
            /* Follow the line using front array PID. (Inverted from - to +) */
            Chassis_Drive(&chassis, base_speed, 0, omega_front);
        }

        /* Wait 1 second before we start looking for a junction (all 8 front sensors see black) */
        if (HAL_GetTick() - state0_start > 1000) {
            if (front_black_count >= 8) {
                robot_state = 1;
                printf("\r\n>>> JUNCTION FOUND! CLEARING IT... <<<\r\n");
            }
        }
    }
    else if (robot_state == 1) {
        /* ====== STATE 1: CLEAR THE JUNCTION ====== */
        /* Start strafing right using the right PID, until the front array leaves the junction */
        Chassis_Drive(&chassis, 0, base_speed, -omega_right);

        if (front_black_count == 0) {
            robot_state = 3;
            printf("\r\n>>> JUNCTION CLEARED. STRAFING... <<<\r\n");
        }
    }
    else if (robot_state == 3) {
        /* ====== STATE 3: STRAFE RIGHT (FOLLOW RIGHT ARRAY) ====== */
        /* Use the right array PID to follow the line while strafing */
        Chassis_Drive(&chassis, 0, base_speed, -omega_right);

        /* When the front array detects a line crossing it again, STOP! */
        if (front_black_count >= 2) {
            Chassis_Drive(&chassis, 0, 0, 0);
            robot_state = 99; // HALT
            printf("\r\n>>> FRONT LINE DETECTED AGAIN - HALTED <<<\r\n");
        }
    }
    else {
        /* HALT state (safety) */
        Chassis_Drive(&chassis, 0, 0, 0);
    }
#endif

    /* ---- TELEMETRY ---- */
#if ENABLE_TELEMETRY
    static uint32_t last_tele = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tele >= 100) {
        const char* front_pins[] = {"PA1", "PA2", "PA3", "PF4", "PA4", "PA5", "PA6", "PA7"};
        const char* right_pins[] = {"PC0", "PC1", "PC5", "PB0", "PB1", "PB2"};

        printf("\033[H");
        printf("==============================\r\n");
        printf("     QTR SENSOR + STATE       \r\n");
        printf("==============================\r\n");

#if ENABLE_QTR_FRONT
        printf("--- FRONT (8) Black:%d ---\r\n", front_black_count);
        for (int i = 0; i < 8; i++) {
            uint16_t raw = adc_front_buffer[i];
            uint16_t cal_val = front_qtr_filtered[i];
            const char* color = (cal_val > 300) ? "BLACK" : "WHITE";
            printf("F%d(%s) Raw:%4d Cal:%3d%% %s\r\n", i, front_pins[i], raw, cal_val/10, color);
        }
#endif
#if ENABLE_QTR_RIGHT
        printf("\r\n--- RIGHT (6) Black:%d ---\r\n", right_black_count);
        for (int i = 0; i < 6; i++) {
            uint16_t raw = adc_right_buffer[i];
            uint16_t cal_val = right_qtr_filtered[i];
            const char* color = (cal_val > 300) ? "BLACK" : "WHITE";
            printf("R%d(%s) Raw:%4d Cal:%3d%% %s\r\n", i, right_pins[i], raw, cal_val/10, color);
        }
#endif

#if ENABLE_CHASSIS
        printf("\r\nState: %d\r\n", robot_state);
#endif
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
