#include "robot_core.h"
#include "line_follower.h"
#include "motor_driver.h"
#include "encoders.h"
#include "imu.h"

#include "qtr_array.h"
#include "sharp_ir.h"
#include "navigation.h"
#include <stdio.h>
#include <math.h>

static uint32_t last_loop_time = 0;

/* Global Control Variables */
float target_heading = 0.0f;
int32_t robot_vx = 0;
int32_t robot_vy = 0;
PID_t gyro_pid;
int32_t nav_omega_override = 0;
bool use_nav_omega = false;

/* Absolute Odometry (Grid Position) */
float global_x = 0.0f; // mm
float global_y = 0.0f; // mm

/* External peripheral handles (defined in main.c) */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;
extern SPI_HandleTypeDef hspi1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;

#if ENABLE_CHASSIS
static Mecanum_Chassis_t chassis;
static LineFollower_t line_follower;
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
    printf("         ROBOT CORE ONLINE            \r\n");
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

    LineFollower_Init(&line_follower);
    LineFollower_Start(&line_follower);
#endif

#if ENABLE_GYRO
    IMU_Init(&hspi1);
    
    /* Initialize Gyro PID: Kp=30, Ki=0, Kd=10, max_int=1000, max_out=1500 */
    PID_Init(&gyro_pid, 30.0f, 0.0f, 10.0f, 0.0f, 1000.0f, 1500.0f);
#endif

    Nav_Init();

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
    IMU_ReadGyro(&hspi1, 0.005f); // Exactly 5ms loop time from TIM6
#endif


#if ENABLE_QTR_ARRAY
    QTR_Poll(&hadc1, &hadc2, &hadc3, &hadc4);
#endif
#if ENABLE_SHARP_IR
    Sharp_Poll(&hadc1);
#endif

    /* Execute line follower instead of navigation */
    LineFollower_Update(&line_follower, HAL_GetTick());

#if ENABLE_CHASSIS
    int32_t left_speed = 0;
    int32_t right_speed = 0;

    LineFollower_GetMotorSpeeds(&line_follower, &left_speed, &right_speed);

    Motor_SetSpeed(&chassis.fl, left_speed);
    Motor_SetSpeed(&chassis.rl, left_speed);
    Motor_SetSpeed(&chassis.fr, right_speed);
    Motor_SetSpeed(&chassis.rr, right_speed);
#endif

#if ENABLE_TELEMETRY
    static uint32_t last_tele = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tele >= 100) {
        printf("\033[H");
        printf("==============================\r\n");
        printf("       ROBOT TELEMETRY        \r\n");
        printf("==============================\r\n");

#if ENABLE_GYRO
        printf("--- GYRO YAW: %.2f deg ---\r\n", (double)gyro_yaw_deg);
#endif

#if ENABLE_ENCODERS
        extern volatile int32_t enc1_count;
        extern volatile int32_t enc2_count;
        extern volatile int32_t enc3_count;
        extern volatile int32_t enc4_count;
        extern int32_t start_ticks;
        extern int32_t target_ticks;
        extern NavState_t nav_state;
        printf("\r\n--- ENCODERS ---\r\n");
        printf("Enc1 (FR): %6ld | Enc2 (RR): %6ld\r\n", enc1_count, enc2_count);
        printf("Enc3 (RL): %6ld | Enc4 (FL): %6ld\r\n", enc3_count, enc4_count);
        printf("Start Ticks: %ld | Target Ticks: %ld | State: %d\r\n", start_ticks, target_ticks, (int)nav_state);
#endif



#if ENABLE_QTR_ARRAY
        printf("\r\n--- QTR ANALOG ARRAYS ---\r\n");
        printf("FRONT: [%4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d] | Pos: %ld\r\n", 
               qtr_front[0], qtr_front[1], qtr_front[2], qtr_front[3], 
               qtr_front[4], qtr_front[5], qtr_front[6], qtr_front[7],
               QTR_GetFrontLinePosition());
        printf("LEFT:  [%4d, %4d, %4d, %4d, %4d, %4d] | Pos: %ld\r\n", 
               qtr_left[0], qtr_left[1], qtr_left[2], qtr_left[3], 
               qtr_left[4], qtr_left[5], QTR_GetLeftLinePosition());
        printf("RIGHT: [%4d, %4d, %4d, %4d, %4d, %4d] | Pos: %ld\r\n", 
               qtr_right[0], qtr_right[1], qtr_right[2], qtr_right[3], 
               qtr_right[4], qtr_right[5], QTR_GetRightLinePosition());

        /* Junction Detection (threshold 4094) */
        bool front_sees_line = false;
        for (int i = 0; i < 8; i++) {
            if (qtr_front[i] >= 4094) {
                front_sees_line = true;
                break;
            }
        }
        bool left_sees_line = false;
        for (int i = 0; i < 6; i++) {
            if (qtr_left[i] >= 4094) {
                left_sees_line = true;
                break;
            }
        }
        /* Right QTR: Skip broken middle pins 2 and 3 */
        bool right_sees_line = (qtr_right[0] >= 4094 || qtr_right[1] >= 4094 || 
                                qtr_right[4] >= 4094 || qtr_right[5] >= 4094);
        bool is_junction = (front_sees_line && left_sees_line && right_sees_line);

        printf("\r\n--- JUNCTION STATUS ---\r\n");
        printf("Front Sees: %s | Left Sees: %s | Right Sees: %s\r\n",
               front_sees_line ? "YES" : "NO",
               left_sees_line ? "YES" : "NO",
               right_sees_line ? "YES" : "NO");
        if (is_junction) {
            printf("junction detected\r\n");
        } else {
            printf("Junction:   NONE\r\n");
        }
#endif

#if ENABLE_SHARP_IR
        printf("\r\n--- DISTANCE ---\r\n");
        printf("SHARP IR (Raw): %4d\r\n", sharp_ir_raw);
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
