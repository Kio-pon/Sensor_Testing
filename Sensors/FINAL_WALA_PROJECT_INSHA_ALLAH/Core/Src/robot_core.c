#include "robot_core.h"
#include "motor_driver.h"
#include "encoders.h"
#include "imu.h"
#include <stdio.h>

static uint32_t last_loop_time = 0;

/* External peripheral handles (defined in main.c) */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern SPI_HandleTypeDef hspi1;

#if ENABLE_CHASSIS
static Mecanum_Chassis_t chassis;
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

/* ============================================================
INIT
============================================================ */

void Robot_Init(void)
{
    printf("\033[2J\033[H");
    printf("======================================\r\n");
    printf("     CLEAN SLATE INIT (PHASE 1)       \r\n");
    printf("======================================\r\n");

#if ENABLE_CHASSIS
    // Reversed FR
    chassis.fr.IN1_Port = GPIOD; chassis.fr.IN1_Pin = GPIO_PIN_1;
    chassis.fr.IN2_Port = GPIOD; chassis.fr.IN2_Pin = GPIO_PIN_2;
    chassis.fr.htim =&htim2; chassis.fr.channel = TIM_CHANNEL_1; chassis.fr.max_pwm=4800;

    // Reversed RR
    chassis.rr.IN1_Port = GPIOC; chassis.rr.IN1_Pin = GPIO_PIN_12;
    chassis.rr.IN2_Port = GPIOD; chassis.rr.IN2_Pin = GPIO_PIN_0;
    chassis.rr.htim =&htim2; chassis.rr.channel = TIM_CHANNEL_2; chassis.rr.max_pwm=4800;

    chassis.rl.IN1_Port = GPIOB; chassis.rl.IN1_Pin = GPIO_PIN_4;
    chassis.rl.IN2_Port = GPIOB; chassis.rl.IN2_Pin = GPIO_PIN_5;
    chassis.rl.htim =&htim2; chassis.rl.channel = TIM_CHANNEL_3; chassis.rl.max_pwm=4800;

    // Reversed FL
    chassis.fl.IN1_Port = GPIOD; chassis.fl.IN1_Pin = GPIO_PIN_9;
    chassis.fl.IN2_Port = GPIOD; chassis.fl.IN2_Pin = GPIO_PIN_8;
    chassis.fl.htim =&htim2; chassis.fl.channel = TIM_CHANNEL_4; chassis.fl.max_pwm=4800;

    chassis.STBY_Port = GPIOD; chassis.STBY_Pin = GPIO_PIN_5;
    Chassis_Init(&chassis);
#endif

#if ENABLE_GYRO
    IMU_Init(&hspi1);
#endif

    /* start TIM6 for encoders and 200 Hz control beat */
    HAL_TIM_Base_Start_IT(&htim6);

    last_loop_time = HAL_GetTick();
}

/* ============================================================
   MAIN LOOP (200 Hz Beat)
   ============================================================ */
void Robot_RunLoop(void)
{
    if (!control_due) return; /* wait for the 200 Hz beat */
    control_due = 0;

#if ENABLE_GYRO
    IMU_ReadGyro(&hspi1);
#endif

#if ENABLE_CHASSIS
    // For Phase 1 testing, just stop motors.
    Chassis_Drive(&chassis, 0, 0, 0); 
#endif

#if ENABLE_TELEMETRY
    static uint32_t last_tele = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tele >= 100) {
        printf("\033[H");
        printf("==============================\r\n");
        printf("     CLEAN SLATE TELEMETRY    \r\n");
        printf("==============================\r\n");

#if ENABLE_GYRO
        printf("--- GYRO YAW: %.2f deg ---\r\n", (double)gyro_yaw_deg);
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
#if ENABLE_ENCODERS
int32_t Strafe_CM_To_Ticks(float cm)
{
    return (int32_t)(cm * TICKS_PER_CM * (100.0f / 84.0f));
}
#endif
#endif
