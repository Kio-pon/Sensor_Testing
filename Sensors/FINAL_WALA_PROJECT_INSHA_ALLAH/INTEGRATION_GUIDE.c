/**
 * ===================================================================
 * INTEGRATION SNIPPET FOR robot_core.c
 * 
 * This file shows EXACTLY what to modify in robot_core.c to
 * integrate the line follower. Copy/paste these sections into
 * your existing robot_core.c file.
 * ===================================================================
 */

/* ===================================================================
   STEP 1: ADD HEADER IN robot_core.c (near top with other includes)
   =================================================================== */

// ADD THIS LINE:
#include "line_follower.h"

/* Your existing includes remain unchanged */


/* ===================================================================
   STEP 2: ADD GLOBAL INSTANCE (after other static globals)
   =================================================================== */

// ADD THIS:
static LineFollower_t line_follower;

/* ... rest of existing globals ... */


/* ===================================================================
   STEP 3: INITIALIZE IN Robot_Init() 
   
   Find Robot_Init() function and add this after Chassis_Init()
   =================================================================== */

void Robot_Init(void) {
    printf("\033[2J\033[H");
    printf("======================================\r\n");
    printf("         ROBOT CORE ONLINE            \r\n");
    printf("======================================\r\n");

#if ENABLE_CHASSIS
    // ... your existing chassis init code ...
    Chassis_Init(&chassis);
#endif

#if ENABLE_GYRO
    IMU_Init(&hspi1);
    PID_Init(&gyro_pid, 30.0f, 0.0f, 10.0f, 0.0f, 1000.0f, 1500.0f);
#endif

    // ======== ADD THIS SECTION ========
    /* Initialize line follower for simple front-only movement */
    LineFollower_Init(&line_follower);
    printf("====== LINE FOLLOWER READY ======\r\n");
    printf("Send 'S' to start, 'X' to stop\r\n\r\n");
    // ==================================

    Nav_Init();
    HAL_TIM_Base_Start_IT(&htim6);
    last_loop_time = HAL_GetTick();
}


/* ===================================================================
   STEP 4: REPLACE MAIN LOOP
   
   This is the most important change. Find the main loop in 
   Robot_RunLoop() and replace the navigation section with this.
   
   The old loop looked something like:
   
   void Robot_RunLoop(void) {
       if (!control_due) return;
       control_due = 0;
       
       QTR_Poll(...);
       Nav_RunSequence();      <-- REMOVE THIS
       Nav_Update();           <-- REMOVE THIS
       ... rest of code ...
   }
   
   Replace with this:
   =================================================================== */

void Robot_RunLoop(void) {
    if (!control_due) return;
    control_due = 0;

#if ENABLE_GYRO
    IMU_ReadGyro(&hspi1, 0.005f);
#endif

    /* ======== LINE FOLLOWING CONTROL ======== */
#if ENABLE_QTR_ARRAY
    QTR_Poll(&hadc1, &hadc2, &hadc3, &hadc4);
    
    /* Update line follower with current time */
    uint32_t current_time = HAL_GetTick();
    LineFollower_Update(&line_follower, current_time);
    
    /* Get the calculated motor speeds */
    int32_t left_motor_speed = 0;
    int32_t right_motor_speed = 0;
    LineFollower_GetMotorSpeeds(&line_follower, 
                                &left_motor_speed, 
                                &right_motor_speed);
    
    /* Apply speeds to all motors (front-left and rear-left are paired,
       same with right pair for skid-steer/mecanum) */
#if ENABLE_CHASSIS
    Motor_SetSpeed(&chassis.fl, left_motor_speed);
    Motor_SetSpeed(&chassis.rl, left_motor_speed);
    Motor_SetSpeed(&chassis.fr, right_motor_speed);
    Motor_SetSpeed(&chassis.rr, right_motor_speed);
#endif
    
#endif
    /* ======== END LINE FOLLOWING ======== */

    /* ======== OPTIONAL TELEMETRY ======== */
#if ENABLE_TELEMETRY
    static uint32_t last_tele = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_tele >= 100) {  /* 100ms = 10 Hz telemetry */
        last_tele = now;
        
        printf("\033[H");
        printf("======== LINE FOLLOWER STATUS ========\r\n");
        
#if ENABLE_QTR_ARRAY
        printf("\nFRONT QTR: [%4d, %4d, %4d, %4d, %4d, %4d, %4d, %4d]\r\n",
               qtr_front[0], qtr_front[1], qtr_front[2], qtr_front[3],
               qtr_front[4], qtr_front[5], qtr_front[6], qtr_front[7]);
        
        int32_t line_pos = QTR_GetFrontLinePosition();
        printf("Line Position: %ld (0=center, ±3500=edge)\r\n", line_pos);
        
        bool detected = LineFollower_IsLineDetected(&line_follower);
        printf("Line Detected: %s\r\n", detected ? "YES" : "NO");
#endif
        
        printf("\nMotor Commands:\r\n");
        printf("  LEFT:  %6ld PWM\r\n", left_motor_speed);
        printf("  RIGHT: %6ld PWM\r\n", right_motor_speed);
        
        printf("\nState: ");
        switch (LineFollower_GetState(&line_follower)) {
            case LF_IDLE:       printf("IDLE"); break;
            case LF_RUNNING:    printf("RUNNING"); break;
            case LF_LINE_LOST:  printf("LINE LOST"); break;
            case LF_FINISHED:   printf("FINISHED"); break;
            default:            printf("UNKNOWN");
        }
        printf("\r\n");
        printf("======================================\r\n\r\n");
    }
#endif
    /* ======== END TELEMETRY ======== */
}


/* ===================================================================
   STEP 5 (OPTIONAL): ADD UART COMMAND HANDLER
   
   If you want to control the line follower via UART (S to start,
   X to stop, + to speed up, etc.), add this to your UART interrupt
   or main loop character processing:
   =================================================================== */

void UART_ProcessCommand(char cmd) {
    /* Existing code probably handles other commands... add this: */
    
    switch (cmd) {
        case 'S':
        case 's':
            LineFollower_Start(&line_follower);
            printf("\r\n[LF] STARTED\r\n");
            break;
        
        case 'X':
        case 'x':
            LineFollower_Stop(&line_follower);
            printf("\r\n[LF] STOPPED\r\n");
            break;
        
        case 'R':
        case 'r':
            LineFollower_Reset(&line_follower);
            printf("\r\n[LF] Reset\r\n");
            break;
        
        case '+': {
            int32_t old_speed = line_follower.config.base_speed;
            line_follower.config.base_speed += 100;
            if (line_follower.config.base_speed > 1800)
                line_follower.config.base_speed = 1800;
            printf("\r\n[LF] Speed: %ld → %ld\r\n", 
                   old_speed, line_follower.config.base_speed);
            break;
        }
        
        case '-': {
            int32_t old_speed = line_follower.config.base_speed;
            line_follower.config.base_speed -= 100;
            if (line_follower.config.base_speed < 400)
                line_follower.config.base_speed = 400;
            printf("\r\n[LF] Speed: %ld → %ld\r\n", 
                   old_speed, line_follower.config.base_speed);
            break;
        }
        
        /* ... handle any other existing commands ... */
        
        default:
            break;
    }
}


/* ===================================================================
   MINIMAL COMPLETE EXAMPLE (for reference)
   
   If you want to start from scratch with just line following:
   =================================================================== */

/*
// Minimal main.c line follower setup:

#include "main.h"
#include "line_follower.h"
#include "qtr_array.h"
#include "motor_driver.h"

static LineFollower_t lf;
static Mecanum_Chassis_t chassis;
extern ADC_HandleTypeDef hadc1, hadc2, hadc3, hadc4;

void setup(void) {
    // Init motors
    chassis.fl.IN1_Port = GPIOD; chassis.fl.IN1_Pin = GPIO_PIN_9;
    // ... configure all 4 motors ...
    Chassis_Init(&chassis);
    
    // Init line follower
    LineFollower_Init(&lf);
    
    // Start timer for control loop
    HAL_TIM_Base_Start_IT(&htim6);  // 200 Hz
}

void control_loop_200hz(void) {
    // Read sensors
    QTR_Poll(&hadc1, &hadc2, &hadc3, &hadc4);
    
    // Update follower
    LineFollower_Update(&lf, HAL_GetTick());
    
    // Get motor speeds
    int32_t left, right;
    LineFollower_GetMotorSpeeds(&lf, &left, &right);
    
    // Send to motors
    Motor_SetSpeed(&chassis.fl, left);
    Motor_SetSpeed(&chassis.rl, left);
    Motor_SetSpeed(&chassis.fr, right);
    Motor_SetSpeed(&chassis.rr, right);
}

// In some init or button handler:
LineFollower_Start(&lf);  // Go!
*/


/* ===================================================================
   TUNING QUICK REFERENCE
   
   After integration, you can adjust these in code or via UART:
   =================================================================== */

/*
// Edit these values to tune:
line_follower.config.base_speed = 1400;        // Speed (PWM)
line_follower.config.max_steering = 600;       // Max turning correction
line_follower.config.kp_steering = 1.2f;       // Responsiveness
line_follower.config.kd_steering = 0.8f;       // Smoothing
line_follower.config.ki_steering = 0.05f;      // Drift correction
line_follower.config.line_threshold = 2000;    // Detection threshold

// Or use the config function:
LineFollower_Config_t cfg = {
    .base_speed = 1300,
    .max_steering = 700,
    .kp_steering = 1.3f,
    .kd_steering = 0.9f,
    // ... etc ...
};
LineFollower_SetConfig(&line_follower, &cfg);
*/


/* ===================================================================
   FILES INVOLVED
   =================================================================== */

/*
Core/Inc/line_follower.h       - Header file (API)
Core/Src/line_follower.c       - Implementation (algorithm)
Core/Src/line_follower_main.c  - Helper functions
Core/Src/robot_core.c          - YOUR FILE (integrate here)
CMakeLists.txt                 - ADD line_follower.c to build
*/


/* ===================================================================
   BUILD CONFIGURATION
   =================================================================== */

/* In CMakeLists.txt, make sure line_follower.c is in target_sources:

target_sources(${TARGET_NAME} PRIVATE
  Core/Src/main.c
  Core/Src/line_follower.c       <-- ADD THIS
  Core/Src/robot_core.c
  Core/Src/motor_driver.c
  Core/Src/qtr_array.c
  # ... other files ...
)

In robot_config.h, ensure:
#define ENABLE_QTR_ARRAY 1        (must be 1)
#define ENABLE_CHASSIS 1          (must be 1)
#define ENABLE_TELEMETRY 1        (optional, for debug)
*/

