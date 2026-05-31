#include "robot_core.h" /* must include robot_config.h */
#include "motor_driver.h"
#include "qtr_8a.h"
#include "pid.h"
#include "tcs34725.h"
#include "servo.h"
#include "encoders.h"
#include <stdio.h>

/* ---- DMA frame sizes = channel counts (single frame, no x2) ---- */
#define ADC1_SIZE 9 /* 8 front QTR + 1 sharp */
#define ADC2_SIZE 6 /* 6 right QTR */
#define ADC3_SIZE 8 /* 6 left QTR + 2 sharp */

static volatile uint16_t adc1_dma_buffer[ADC1_SIZE] = {0};
static volatile uint16_t adc2_dma_buffer[ADC2_SIZE] = {0};
static volatile uint16_t adc3_dma_buffer[ADC3_SIZE] = {0};

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

#if ENABLE_QTR_FRONT
static QTR_Array_t qtr_front;
static PID_t line_pid;
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

#if ENABLE_SHARP_FRONT || ENABLE_SHARP_LEFT || ENABLE_SHARP_RIGHT
static volatile float sharp_cm_front = 150.0f;
static volatile float sharp_cm_left  = 150.0f;
static volatile float sharp_cm_right = 150.0f;

static float Calculate_Sharp_CM(uint16_t raw_adc)
{
    float voltage = (float)raw_adc * 3.3f / 4095.0f;
    if (voltage < 0.35f) return 150.0f; /* far/no echo */
    if (voltage > 2.6f) return 15.0f; /* clamp at blind floor */
    return 62.28f / (voltage - 0.02f);
}

/* light EMA so the distance does not jitter the control loop */
static float Sharp_Filter(float prev, uint16_t raw)
{
    float cm = Calculate_Sharp_CM(raw);
    return (EMA_ALPHA * cm) + ((1.0f - EMA_ALPHA) * prev);
}
#endif

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


/* ============================================================
INIT
Correct order:
1) hardware ADC self-calibration (BEFORE any DMA)
2) start DMA
3) QTR_Init with NULL handles (no internal polling)
4) software sweep reading DMA buffers only
5) start TIM6
============================================================ */
void Robot_Init(void)
{
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

    /* 1) hardware self-calibration BEFORE DMA. Never on a running ADC. */
#if ENABLE_QTR_FRONT || ENABLE_SHARP_FRONT
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
#endif
#if ENABLE_QTR_RIGHT
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
#endif
#if ENABLE_QTR_LEFT || ENABLE_SHARP_LEFT || ENABLE_SHARP_RIGHT
    HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);
#endif

    /* 2) start DMA (single frame, size == channel count) */
#if ENABLE_QTR_FRONT || ENABLE_SHARP_FRONT
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_dma_buffer, ADC1_SIZE);
#endif
#if ENABLE_QTR_RIGHT
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_dma_buffer, ADC2_SIZE);
#endif
#if ENABLE_QTR_LEFT || ENABLE_SHARP_LEFT || ENABLE_SHARP_RIGHT
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_dma_buffer, ADC3_SIZE);
#endif

    HAL_Delay(5); /* let the first frames land */

    /* 3) QTR_Init with NULL handles: skips all internal polling */
#if ENABLE_QTR_FRONT
    QTR_Init(&qtr_front, NULL, NULL, 8, 500);
    PID_Init(&line_pid, 0.25f, 0.0f, 0.15f, 500.0f, 1200.0f);
#if ENABLE_CHASSIS
    /* 4) sweep reads adc1_dma_buffer only, no polling */
    QTR_CalibrateSensorSweep(&qtr_front, (uint16_t*)adc1_dma_buffer, 5000, &chassis);
#endif
#endif

#if ENABLE_QTR_RIGHT
    QTR_Init(&qtr_right, NULL, NULL, 6, 500);
#if ENABLE_CHASSIS
    QTR_CalibrateSensorSweep(&qtr_right, (uint16_t*)adc2_dma_buffer, 5000, &chassis);
#endif
#endif

#if ENABLE_QTR_LEFT
    QTR_Init(&qtr_left, NULL, NULL, 6, 500);
#if ENABLE_CHASSIS
    QTR_CalibrateSensorSweep(&qtr_left, (uint16_t*)adc3_dma_buffer, 5000, &chassis);
#endif
#endif

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
    int32_t base_speed = 800;
    uint8_t force_brake = 0;

#if ENABLE_QTR_FRONT
    /* read raw DMA frame, calibrate, filter, all at 200 Hz */
    uint16_t cal[8];
    QTR_ReadCalibrated(&qtr_front, cal, (uint16_t*)&adc1_dma_buffer[0]);
    for (int i = 0; i < 8; i++)
        front_qtr_filtered[i] =
            (uint16_t)((EMA_ALPHA * cal[i]) + ((1.0f - EMA_ALPHA) * front_qtr_filtered[i]));
#if ENABLE_SHARP_FRONT
    sharp_cm_front = Sharp_Filter(sharp_cm_front, adc1_dma_buffer[8]);
#endif

    float position = QTR_GetLinePosition(&qtr_front, (uint16_t*)front_qtr_filtered);
    float error = LINE_CENTER - position;
    if (error < -LINE_DEADBAND || error > LINE_DEADBAND)
        omega = (int32_t)PID_Update(&line_pid, error, dt);
    else
        PID_Reset(&line_pid);

    uint8_t all_white = 1;
    for (int i = 0; i < 8; i++)
        if (front_qtr_filtered[i] >= 300) { all_white = 0; break; }
    if (all_white) force_brake = 1;
#endif

#if ENABLE_QTR_RIGHT
    {
        uint16_t calr[6];
        QTR_ReadCalibrated(&qtr_right, calr, (uint16_t*)&adc2_dma_buffer[0]);
        for (int i = 0; i < 6; i++)
            right_qtr_filtered[i] =
                (uint16_t)((EMA_ALPHA * calr[i]) + ((1.0f - EMA_ALPHA) * right_qtr_filtered[i]));
    }
#endif

#if ENABLE_QTR_LEFT
    {
        uint16_t call[6];
        QTR_ReadCalibrated(&qtr_left, call, (uint16_t*)&adc3_dma_buffer[0]);
        for (int i = 0; i < 6; i++)
            left_qtr_filtered[i] =
                (uint16_t)((EMA_ALPHA * call[i]) + ((1.0f - EMA_ALPHA) * left_qtr_filtered[i]));
#if ENABLE_SHARP_LEFT
        sharp_cm_left = Sharp_Filter(sharp_cm_left, adc3_dma_buffer[6]);
#endif
#if ENABLE_SHARP_RIGHT
        sharp_cm_right = Sharp_Filter(sharp_cm_right, adc3_dma_buffer[7]);
#endif
    }
#endif

#if ENABLE_CHASSIS
#if ENABLE_QTR_FRONT
    if (force_brake) Chassis_Drive(&chassis, 0, 0, 0);
    else Chassis_Drive(&chassis, base_speed, 0, omega);
#else
    Chassis_Drive(&chassis, 0, 0, 0);
#endif
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
        printf("LINE [");
        for (int i = 0; i < 8; i++)
            printf(front_qtr_filtered[i] >= 500 ? "#" :
                  (front_qtr_filtered[i] >= 200 ? "-" : " "));
        printf("]\r\n");
#endif
#if ENABLE_SHARP_FRONT
        printf("FRONT : %6.1f cm\r\n", (double)sharp_cm_front);
#endif
        printf("==============================\r\n");
        last_tele = now;
    }
#endif
}

/* ============================================================
   TURN ROUTINES
   ============================================================ */

#define TURN_SPEED_FAST 800
#define TURN_SPEED_SLOW 400
#define LINE_ON 500 /* calibrated value: clearly on the line */
#define LINE_OFF 200 /* calibrated value: clearly off the line */
#define TURN_TIMEOUT_MS 3000

/* strongest of the two center sensors, from a fresh raw frame */
static uint16_t Front_CenterStrength(void)
{
    uint16_t cal[8];
    QTR_ReadCalibrated(&qtr_front, cal, (uint16_t*)&adc1_dma_buffer[0]);
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
