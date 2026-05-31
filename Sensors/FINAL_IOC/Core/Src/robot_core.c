#include "robot_core.h"
#include "motor_driver.h"
#include "qtr_8a.h"
#include "pid.h"
#include "tcs34725.h"
#include "servo.h"
#include <stdio.h>

#define ADC1_SIZE 9  // 8 Front QTR + 1 Sharp
#define ADC2_SIZE 6  // 6 Right QTR
#define ADC3_SIZE 8  // 6 Left QTR + 2 Sharps

static volatile uint16_t adc1_dma_buffer[ADC1_SIZE * 2] = {0};
static volatile uint16_t adc2_dma_buffer[ADC2_SIZE * 2] = {0};
static volatile uint16_t adc3_dma_buffer[ADC3_SIZE * 2] = {0};

static const float EMA_ALPHA = 0.25f;
static uint32_t last_loop_time = 0;
static uint32_t last_telemetry_time = 0;

#if ENABLE_CHASSIS
static Mecanum_Chassis_t chassis;
#endif

/* External peripheral handles (defined in main.c) */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c1;

#if ENABLE_QTR_FRONT
static ADC_HandleTypeDef *qtr_front_hadcs[8] = {
    &hadc1, &hadc1, &hadc1, &hadc1,
    &hadc1, &hadc1, &hadc1, &hadc1
};
static const uint32_t qtr_front_channels[8] = {
    ADC_CHANNEL_2,
    ADC_CHANNEL_3,
    ADC_CHANNEL_4,
    ADC_CHANNEL_6,
    ADC_CHANNEL_7,
    ADC_CHANNEL_8,
    ADC_CHANNEL_9,
    ADC_CHANNEL_5
};
static QTR_Array_t qtr_front;
static PID_t line_pid;
static volatile uint16_t front_qtr_filtered[8] = {0};
#endif

#if ENABLE_QTR_LEFT
static ADC_HandleTypeDef *qtr_left_hadcs[6] = { &hadc3, &hadc3, &hadc3, &hadc3, &hadc3, &hadc3 };
static const uint32_t qtr_left_channels[6] = { ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_5, ADC_CHANNEL_6, ADC_CHANNEL_7 };
static QTR_Array_t qtr_left;
static volatile uint16_t left_qtr_filtered[6] = {0};
#endif

#if ENABLE_QTR_RIGHT
static ADC_HandleTypeDef *qtr_right_hadcs[6] = { &hadc2, &hadc2, &hadc2, &hadc2, &hadc2, &hadc2 };
static const uint32_t qtr_right_channels[6] = { ADC_CHANNEL_1, ADC_CHANNEL_2, ADC_CHANNEL_3, ADC_CHANNEL_4, ADC_CHANNEL_11, ADC_CHANNEL_12 };
static QTR_Array_t qtr_right;
static volatile uint16_t right_qtr_filtered[6] = {0};
#endif

#if ENABLE_SHARP_FRONT || ENABLE_SHARP_LEFT || ENABLE_SHARP_RIGHT
static volatile float sharp_cm_front = 150.0f;
static volatile float sharp_cm_left  = 150.0f;
static volatile float sharp_cm_right = 150.0f;

static float Calculate_Sharp_CM(uint16_t raw_adc)
{
    float voltage = (float)raw_adc * 3.3f / 4095.0f;
    if (voltage < 0.35f) return 150.0f;
    if (voltage > 2.6f) return 15.0f;
    return 62.28f / (voltage - 0.02f);
}
#endif

#if ENABLE_COLOR_SENSOR
static TCS34725_RawData color_raw;
static DetectedColor color_result;
#endif

#if ENABLE_ARM_SERVO
static Elevator_t arm_elevator;
#endif

#if ENABLE_ENCODERS
#include "encoders.h"

static const int8_t Q_TABLE[4][4] = {
    { 0, -1,  1,  0 },
    { 1,  0,  0, -1 },
    {-1,  0,  0,  1 },
    { 0,  1, -1,  0 }
};

static void Poll_Encoders(void)
{
    static uint8_t p1 = 0, p2 = 0, p3 = 0, p4 = 0, init = 0;
    if (!init) {
        p1 = (uint8_t)((GPIOA->IDR >> 9) & 3);
        p2 = (uint8_t)((((GPIOA->IDR >> 8) & 1) << 1) | ((GPIOC->IDR >> 9) & 1));
        p3 = (uint8_t)((GPIOC->IDR >> 7) & 3);
        p4 = (uint8_t)((((GPIOC->IDR >> 6) & 1) << 1) | ((GPIOD->IDR >> 15) & 1));
        init = 1;
        return;
    }

    uint8_t c1 = (uint8_t)((GPIOA->IDR >> 9) & 3);
    if (c1 != p1) { enc1_count += Q_TABLE[p1][c1]; p1 = c1; }

    uint8_t c2 = (uint8_t)((((GPIOA->IDR >> 8) & 1) << 1) | ((GPIOC->IDR >> 9) & 1));
    if (c2 != p2) { enc2_count -= Q_TABLE[p2][c2]; p2 = c2; }

    uint8_t c3 = (uint8_t)((GPIOC->IDR >> 7) & 3);
    if (c3 != p3) { enc3_count -= Q_TABLE[p3][c3]; p3 = c3; }

    uint8_t c4 = (uint8_t)((((GPIOC->IDR >> 6) & 1) << 1) | ((GPIOD->IDR >> 15) & 1));
    if (c4 != p4) { enc4_count += Q_TABLE[p4][c4]; p4 = c4; }
}
#endif

void Robot_Init(void)
{
    printf("\033[2J\033[H");
    printf("======================================\r\n");
    printf("   NERC ROBOT MODULAR CORE BOOTING    \r\n");
    printf("======================================\r\n");

#if ENABLE_CHASSIS
    chassis.fr.IN1_Port = GPIOD;
    chassis.fr.IN1_Pin  = GPIO_PIN_2;
    chassis.fr.IN2_Port = GPIOD;
    chassis.fr.IN2_Pin  = GPIO_PIN_1;
    chassis.fr.htim     = &htim2;
    chassis.fr.channel  = TIM_CHANNEL_1;
    chassis.fr.max_pwm  = 4800;

    chassis.rr.IN1_Port = GPIOD;
    chassis.rr.IN1_Pin  = GPIO_PIN_0;
    chassis.rr.IN2_Port = GPIOC;
    chassis.rr.IN2_Pin  = GPIO_PIN_12;
    chassis.rr.htim     = &htim2;
    chassis.rr.channel  = TIM_CHANNEL_2;
    chassis.rr.max_pwm  = 4800;

    chassis.rl.IN1_Port = GPIOB;
    chassis.rl.IN1_Pin  = GPIO_PIN_14;
    chassis.rl.IN2_Port = GPIOB;
    chassis.rl.IN2_Pin  = GPIO_PIN_15;
    chassis.rl.htim     = &htim2;
    chassis.rl.channel  = TIM_CHANNEL_3;
    chassis.rl.max_pwm  = 4800;

    chassis.fl.IN1_Port = GPIOD;
    chassis.fl.IN1_Pin  = GPIO_PIN_8;
    chassis.fl.IN2_Port = GPIOD;
    chassis.fl.IN2_Pin  = GPIO_PIN_9;
    chassis.fl.htim     = &htim2;
    chassis.fl.channel  = TIM_CHANNEL_4;
    chassis.fl.max_pwm  = 4800;

    chassis.STBY_Port = GPIOD;
    chassis.STBY_Pin  = GPIO_PIN_5;

    printf("  [Robot_Init] Initializing chassis...\r\n");
    Chassis_Init(&chassis);
    printf("  [Robot_Init] Chassis initialized.\r\n");
#endif

#if ENABLE_QTR_FRONT || ENABLE_SHARP_FRONT
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_dma_buffer, ADC1_SIZE * 2);
#endif
#if ENABLE_QTR_RIGHT
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_dma_buffer, ADC2_SIZE * 2);
#endif
#if ENABLE_QTR_LEFT || ENABLE_SHARP_LEFT || ENABLE_SHARP_RIGHT
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_dma_buffer, ADC3_SIZE * 2);
#endif

#if ENABLE_QTR_FRONT
    printf("  [Robot_Init] Initializing front QTR...\r\n");
    QTR_Init(&qtr_front, qtr_front_hadcs, qtr_front_channels, 8, 500);
    PID_Init(&line_pid, 0.6f, 0.0f, 0.05f, 500.0f, 1200.0f);
    #if ENABLE_CHASSIS
    QTR_CalibrateSensorSweep(&qtr_front, (uint16_t*)adc1_dma_buffer, 5000, &chassis);
    #endif
#endif

#if ENABLE_QTR_RIGHT
    printf("  [Robot_Init] Initializing right QTR...\r\n");
    QTR_Init(&qtr_right, qtr_right_hadcs, qtr_right_channels, 6, 500);
    #if ENABLE_CHASSIS
    QTR_CalibrateSensorSweep(&qtr_right, (uint16_t*)adc2_dma_buffer, 5000, &chassis);
    #endif
#endif

#if ENABLE_QTR_LEFT
    printf("  [Robot_Init] Initializing left QTR...\r\n");
    QTR_Init(&qtr_left, qtr_left_hadcs, qtr_left_channels, 6, 500);
    #if ENABLE_CHASSIS
    QTR_CalibrateSensorSweep(&qtr_left, (uint16_t*)adc3_dma_buffer, 5000, &chassis);
    #endif
#endif

#if ENABLE_COLOR_SENSOR
    if (TCS34725_Init(&hi2c1) == HAL_OK) {
        printf("  [Robot_Init] TCS34725 color sensor ready.\r\n");
    } else {
        printf("  [Robot_Init] TCS34725 color sensor failed.\r\n");
    }
#endif

#if ENABLE_ARM_SERVO
    Elevator_Init(&arm_elevator, &htim3, TIM_CHANNEL_1);
#endif

    last_loop_time = HAL_GetTick();
    last_telemetry_time = last_loop_time;
}

void Robot_RunLoop(void)
{
    uint32_t current_time = HAL_GetTick();
    float dt = (float)(current_time - last_loop_time) / 1000.0f;
    if (dt <= 0.0f) dt = 0.001f;
    last_loop_time = current_time;

    int32_t omega = 0;
    int32_t base_speed = 1000;
    uint8_t force_brake = 0;

#if ENABLE_ENCODERS
    Poll_Encoders();
#endif

#if ENABLE_SHARP_FRONT
    if (sharp_cm_front < 15.0f) force_brake = 1;
#endif
#if ENABLE_SHARP_LEFT
    if (sharp_cm_left < 15.0f) force_brake = 1;
#endif
#if ENABLE_SHARP_RIGHT
    if (sharp_cm_right < 15.0f) force_brake = 1;
#endif

#if ENABLE_QTR_FRONT
    {
        float position = QTR_GetLinePosition(&qtr_front, (uint16_t*)front_qtr_filtered);
        float error = 3500.0f - position;
        if (error < -100.0f || error > 100.0f) {
            omega = (int32_t)PID_Update(&line_pid, error, dt);
        } else {
            PID_Reset(&line_pid);
        }

        uint8_t all_white = 1;
        for (int i = 0; i < 8; i++) {
            if (front_qtr_filtered[i] >= 300) {
                all_white = 0;
                break;
            }
        }
        if (all_white) force_brake = 1;
    }
#else
    (void)omega;
#endif

#if ENABLE_COLOR_SENSOR
    if ((current_time % 50) == 0) {
        TCS34725_ReadRaw(&hi2c1, &color_raw);
        color_result = TCS34725_ClassifyColor(&color_raw);
    }
#endif

#if ENABLE_CHASSIS
    if (force_brake || !ENABLE_QTR_FRONT) {
        Chassis_Drive(&chassis, 0, 0, 0);
    } else {
        Chassis_Drive(&chassis, base_speed, 0, omega);
    }
#endif

#if ENABLE_TELEMETRY
    if (current_time - last_telemetry_time >= 100) {
        printf("\033[H");
        printf("======================================\r\n");
        printf("       MODULAR ROBOT DASHBOARD        \r\n");
        printf("======================================\r\n");

#if ENABLE_QTR_FRONT
        {
            float position = QTR_GetLinePosition(&qtr_front, (uint16_t*)front_qtr_filtered);
            printf(" [FRONT QTR MODULE ACTIVE]\r\n");
            printf(" POSITION : %6.1f / 7000\r\n", (double)position);
            printf(" OMEGA    : %6ld\r\n", (long)omega);
            printf(" SENSORS  : [");
            for (int i = 0; i < 8; i++) {
                printf(front_qtr_filtered[i] >= 500 ? "#" : (front_qtr_filtered[i] >= 200 ? "-" : " "));
            }
            printf("]\r\n");
        }
#endif

#if ENABLE_SHARP_FRONT
        printf("--------------------------------------\r\n");
        printf(" [FRONT SHARP DISTANCE]\r\n");
        printf(" DISTANCE : %6.1f cm\r\n", (double)sharp_cm_front);
#endif

#if ENABLE_SHARP_LEFT
        printf("--------------------------------------\r\n");
        printf(" [LEFT SHARP DISTANCE]\r\n");
        printf(" DISTANCE : %6.1f cm\r\n", (double)sharp_cm_left);
#endif

#if ENABLE_SHARP_RIGHT
        printf("--------------------------------------\r\n");
        printf(" [RIGHT SHARP DISTANCE]\r\n");
        printf(" DISTANCE : %6.1f cm\r\n", (double)sharp_cm_right);
#endif

#if ENABLE_COLOR_SENSOR
        printf("--------------------------------------\r\n");
        printf(" [COLOR SENSOR ACTIVE]\r\n");
        printf(" R:%4d G:%4d B:%4d C:%4d\r\n", color_raw.r, color_raw.g, color_raw.b, color_raw.c);
        printf(" DETECTED : %d\r\n", (int)color_result);
#endif

        printf("======================================\r\n");
        last_telemetry_time = current_time;
    }
#endif
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
#if ENABLE_QTR_FRONT
        uint16_t calibrated[8];
        QTR_ReadCalibrated(&qtr_front, calibrated, (uint16_t*)&adc1_dma_buffer[0]);
        for (int i = 0; i < 8; i++) {
            front_qtr_filtered[i] = (uint16_t)((EMA_ALPHA * calibrated[i]) + ((1.0f - EMA_ALPHA) * front_qtr_filtered[i]));
        }
#endif
#if ENABLE_SHARP_FRONT
        sharp_cm_front = Calculate_Sharp_CM(adc1_dma_buffer[8]);
#endif
    }

    if (hadc->Instance == ADC2) {
#if ENABLE_QTR_RIGHT
        uint16_t calibrated[6];
        QTR_ReadCalibrated(&qtr_right, calibrated, (uint16_t*)&adc2_dma_buffer[0]);
        for (int i = 0; i < 6; i++) {
            right_qtr_filtered[i] = (uint16_t)((EMA_ALPHA * calibrated[i]) + ((1.0f - EMA_ALPHA) * right_qtr_filtered[i]));
        }
#endif
    }

    if (hadc->Instance == ADC3) {
#if ENABLE_QTR_LEFT
        uint16_t calibrated[6];
        QTR_ReadCalibrated(&qtr_left, calibrated, (uint16_t*)&adc3_dma_buffer[0]);
        for (int i = 0; i < 6; i++) {
            left_qtr_filtered[i] = (uint16_t)((EMA_ALPHA * calibrated[i]) + ((1.0f - EMA_ALPHA) * left_qtr_filtered[i]));
        }
#endif
#if ENABLE_SHARP_LEFT
        sharp_cm_left = Calculate_Sharp_CM(adc3_dma_buffer[6]);
#endif
#if ENABLE_SHARP_RIGHT
        sharp_cm_right = Calculate_Sharp_CM(adc3_dma_buffer[7]);
#endif
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
#if ENABLE_QTR_FRONT
        uint16_t calibrated[8];
        QTR_ReadCalibrated(&qtr_front, calibrated, (uint16_t*)&adc1_dma_buffer[ADC1_SIZE]);
        for (int i = 0; i < 8; i++) {
            front_qtr_filtered[i] = (uint16_t)((EMA_ALPHA * calibrated[i]) + ((1.0f - EMA_ALPHA) * front_qtr_filtered[i]));
        }
#endif
#if ENABLE_SHARP_FRONT
        sharp_cm_front = Calculate_Sharp_CM(adc1_dma_buffer[ADC1_SIZE + 8]);
#endif
    }

    if (hadc->Instance == ADC2) {
#if ENABLE_QTR_RIGHT
        uint16_t calibrated[6];
        QTR_ReadCalibrated(&qtr_right, calibrated, (uint16_t*)&adc2_dma_buffer[ADC2_SIZE]);
        for (int i = 0; i < 6; i++) {
            right_qtr_filtered[i] = (uint16_t)((EMA_ALPHA * calibrated[i]) + ((1.0f - EMA_ALPHA) * right_qtr_filtered[i]));
        }
#endif
    }

    if (hadc->Instance == ADC3) {
#if ENABLE_QTR_LEFT
        uint16_t calibrated[6];
        QTR_ReadCalibrated(&qtr_left, calibrated, (uint16_t*)&adc3_dma_buffer[ADC3_SIZE]);
        for (int i = 0; i < 6; i++) {
            left_qtr_filtered[i] = (uint16_t)((EMA_ALPHA * calibrated[i]) + ((1.0f - EMA_ALPHA) * left_qtr_filtered[i]));
        }
#endif
#if ENABLE_SHARP_LEFT
        sharp_cm_left = Calculate_Sharp_CM(adc3_dma_buffer[ADC3_SIZE + 6]);
#endif
#if ENABLE_SHARP_RIGHT
        sharp_cm_right = Calculate_Sharp_CM(adc3_dma_buffer[ADC3_SIZE + 7]);
#endif
    }
}
