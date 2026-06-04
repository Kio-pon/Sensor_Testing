#include "robot_core.h" /* must include robot_config.h */
#include "motor_driver.h"
#include "pid.h"
#include "tcs34725.h"
#include "servo.h"
#include "encoders.h"
#include <stdio.h>
#include "imu.h"

/* Distance Sensors */
#include "VL53L0X.h"
#include "sharp_sensor.h"

extern SPI_HandleTypeDef hspi1;
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

#if ENABLE_BFD_FRONT
static volatile uint8_t bfd_front[5] = {0};
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

    uint8_t c1 = rd_enc1(); if(c1 != p1) { enc1_count -= Q_TABLE[p1][c1]; p1 = c1; }
    uint8_t c2 = rd_enc2(); if(c2 != p2) { enc2_count += Q_TABLE[p2][c2]; p2 = c2; }
    uint8_t c3 = rd_enc3(); if(c3 != p3) { enc3_count += Q_TABLE[p3][c3]; p3 = c3; }
    uint8_t c4 = rd_enc4(); if(c4 != p4) { enc4_count -= Q_TABLE[p4][c4]; p4 = c4; }
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



#define BFD_BLACK GPIO_PIN_RESET
#define BFD_WHITE GPIO_PIN_SET

SharpSensor_t sharp_front;

static void Sensors_Poll(void)
{
    /* Digital BFD-1000 reads */
#if ENABLE_BFD_FRONT
    bfd_front[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1); // PA1
    bfd_front[1] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2); // PA2
    bfd_front[2] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3); // PA3
    bfd_front[3] = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4); // PF4
    bfd_front[4] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4); // PA4
#endif

#if ENABLE_SHARP_FRONT
    sharp_cm_front = SharpSensor_ReadDistance(&sharp_front);
#endif

#if ENABLE_SHARP_RIGHT
    uint16_t sharp_raw_r = Read_ADC_Channel(&hadc2, ADC_CHANNEL_15);
    sharp_cm_right = Calculate_Sharp_CM(sharp_raw_r);
#endif

#if ENABLE_SHARP_LEFT
    uint16_t sharp_raw_l = Read_ADC_Channel(&hadc3, ADC_CHANNEL_15);
    sharp_cm_left = Calculate_Sharp_CM(sharp_raw_l);
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

    // Configure BFD-1000 pins as INPUT
    GPIO_InitTypeDef GPIO_InitStruct_BFD = {0};
    GPIO_InitStruct_BFD.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct_BFD.Pull = GPIO_NOPULL;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct_BFD.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_BFD);

    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitStruct_BFD.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct_BFD);

    // Calibrate only ADC4 which is used for the Sharp sensor
    HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);

#if ENABLE_SHARP_FRONT
    SharpSensor_Init(&sharp_front, &hadc4, ADC_CHANNEL_4); // PB14 (ADC4_IN4)
#endif

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
    
    // Configure XSHUT pins as Outputs!
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct_VL = {0};
    GPIO_InitStruct_VL.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct_VL.Pull = GPIO_NOPULL;
    GPIO_InitStruct_VL.Speed = GPIO_SPEED_FREQ_LOW;
    
    GPIO_InitStruct_VL.Pin = GPIO_PIN_2; // FRONT
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct_VL);
    
    GPIO_InitStruct_VL.Pin = GPIO_PIN_11 | GPIO_PIN_12; // LEFT & RIGHT
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct_VL);

    // 1. Pull XSHUT low to reset
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET); // FRONT
    HAL_Delay(10);
    
#if ENABLE_VL53L0X_FRONT
    printf("Booting FRONT VL53L0X...\r\n");
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET); // Boot FRONT
    HAL_Delay(10);
    VL53L0X_SelectSensor(0);
    setTimeout(500); // 500ms timeout to prevent infinite hangs!
    
    // We skip setAddress() so it stays at the default 0x52!
    // printf("Setting I2C Address...\r\n");
    // setAddress(0x54);

    printf("Initializing VL53L0X...\r\n");
    if (!initVL53L0X(1)) printf("VL53L0X FRONT Init Failed! (Check wiring/XSHUT)\r\n");
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
    PID_Init(&line_pid_front, 6.0f, 0.0f, 3.0f, 500.0f, 4800.0f);
    PID_Init(&line_pid_right, 6.0f, 0.0f, 3.0f, 500.0f, 4800.0f);
    PID_Init(&line_pid_left,  6.0f, 0.0f, 3.0f, 500.0f, 4800.0f);
#endif

#if ENABLE_CHASSIS
#endif

#if ENABLE_GYRO
    IMU_Init(&hspi1);
#endif

#if ENABLE_QTR_RIGHT
    QTR_Init(&qtr_right, 6);
#endif
#if ENABLE_QTR_LEFT
    QTR_Init(&qtr_left, 6);
    PID_Init(&line_pid_left, 0.80f, 0.0f, 0.00f, 500.0f, 1200.0f);
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

#if ENABLE_GYRO
    IMU_ReadGyro(&hspi1);
#endif

    int32_t omega_front = 0;
    uint8_t front_black_count = 0;

    /* Manually poll all ADCs for this frame */
    Sensors_Poll();

    /* ---- FRONT ARRAY PROCESSING ---- */
#if ENABLE_BFD_FRONT
    {
        front_black_count = 0;
        for (int i = 0; i < 5; i++) {
            if (bfd_front[i] == BFD_BLACK) {
                front_black_count++;
            }
        }

        if (front_black_count == 0) {
            // Lost line
            omega_front = 0;
            PID_Reset(&line_pid_front);
        } else if (front_black_count >= 4) {
            // Junction detected
            omega_front = 0;
            PID_Reset(&line_pid_front);
        } else {
            // Custom weighting logic for 5-channel BFD-1000
            // bfd_front[2] (S3 - Middle) = 0 weight
            // bfd_front[1] (S2 - Inner Left) = -1.0
            // bfd_front[0] (S1 - Far Left) = -3.0
            // bfd_front[3] (S4 - Inner Right) = +1.0
            // bfd_front[4] (S5 - Far Right) = +3.0
            
            float error = 0.0f;
            float total_weight = 0.0f;
            
            if (bfd_front[0] == BFD_BLACK) { error += -3.0f; total_weight += 1.0f; }
            if (bfd_front[1] == BFD_BLACK) { error += -1.0f; total_weight += 1.0f; }
            if (bfd_front[2] == BFD_BLACK) { error +=  0.0f; total_weight += 1.0f; }
            if (bfd_front[3] == BFD_BLACK) { error +=  1.0f; total_weight += 1.0f; }
            if (bfd_front[4] == BFD_BLACK) { error +=  3.0f; total_weight += 1.0f; }
            
            if (total_weight > 0.0f) {
                error = error / total_weight; // Average the active weights
            }
            
            // Multiply by 1000 to keep the PID constants similar to the old QTR values
            omega_front = (int32_t)PID_Update(&line_pid_front, error * 1000.0f, dt);
        }
    }
#endif

    /* ---- JUNCTION CHECKING (NO MOTORS) ---- */
    if (front_black_count >= 4) {
        printf("\r\n============================\r\n");
        printf("     !!! CROSS JUNCTION !!! \r\n");
        printf("============================\r\n");
    }

    /* ---- STATE MACHINE ---- */
#if ENABLE_CHASSIS
    int32_t current_speed = 2400; // Base speed

    // Drive forward constantly. If no line is detected, omega_front is 0, so it drives straight.
    Chassis_Drive(&chassis, current_speed, 0, omega_front); 
#endif

    /* ---- TELEMETRY ---- */
#if ENABLE_TELEMETRY
    static uint32_t last_tele = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tele >= 100) {
        printf("\033[H");
        printf("==============================\r\n");
        printf("     BFD SENSOR + STATE       \r\n");
        printf("==============================\r\n");

#if ENABLE_GYRO
#if ENABLE_CHASSIS
        printf("--- GYRO YAW: %.2f deg ---\r\n", (double)gyro_yaw_deg);
#else
        printf("--- GYRO YAW: %.2f deg ---\r\n", (double)gyro_yaw_deg);
#endif
#endif

#if ENABLE_VL53L0X_FRONT
        VL53L0X_SelectSensor(0);
        uint16_t dist_front = readRangeContinuousMillimeters(NULL);
        printf("--- FRONT VL53L0X: %u mm ---\r\n", dist_front);
#endif


#if ENABLE_BFD_FRONT
        printf("--- FRONT BFD DIGITAL (0=BLACK) ---\r\n");
        printf("S1:%d | S2:%d | S3:%d | S4:%d | S5:%d\r\n", 
               bfd_front[0], bfd_front[1], bfd_front[2], bfd_front[3], bfd_front[4]);
#endif

#if ENABLE_SHARP_FRONT
        printf("--------------------------------------\r\n");
        printf(" [FRONT SHARP DISTANCE]\r\n");
        printf(" DISTANCE : %6.1f cm\r\n", (double)sharp_cm_front);
#endif

#if ENABLE_CHASSIS
        // printf("\r\nChassis Active\r\n");
#endif

#if ENABLE_ENCODERS
        extern volatile int32_t enc1_count;
        extern volatile int32_t enc2_count;
        extern volatile int32_t enc3_count;
        extern volatile int32_t enc4_count;
        printf("\r\n--- ENCODERS ---\r\n");
        printf("Enc1 (FR): %6ld | Enc2 (RR): %6ld\r\n", enc1_count, enc2_count);
        printf("Enc3 (RL): %6ld | Enc4 (FL): %6ld\r\n", enc3_count, enc4_count);
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

int32_t Strafe_CM_To_Ticks(float cm)
{
    // The user commanded 100cm but the robot only physically moved 84cm.
    // 100 / 84 = 1.1904. We multiply the expected ticks by this ratio to compensate.
    return (int32_t)(cm * TICKS_PER_CM * (100.0f / 84.0f));
}
#endif // ENABLE_ENCODERS

#endif /* ENABLE_CHASSIS */
