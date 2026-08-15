/**
 * ===================================================================
 * SIMPLE LINE FOLLOWER MAIN LOOP
 * 
 * Usage:
 * 1. Include this setup in your main.c initialization
 * 2. Call LineFollower_Update() in your 200 Hz control loop
 * 3. Send motor speeds to chassis using LineFollower_GetMotorSpeeds()
 * 
 * To integrate into existing robot_core.c:
 * - Add #include "line_follower.h" 
 * - Create global: LineFollower_t line_follower;
 * - In Robot_Init(), call: LineFollower_Init(&line_follower);
 * - In Robot_RunLoop() main section, replace navigation with:
 *     LineFollower_Update(&line_follower, HAL_GetTick());
 *     LineFollower_GetMotorSpeeds(&line_follower, &motor_left, &motor_right);
 * ===================================================================
 */

#include "line_follower.h"
#include "qtr_array.h"
#include "motor_driver.h"
#include <stdio.h>

/* Create a single global line follower instance */
static LineFollower_t line_follower;

/**
 * @brief Initialize line follower in your Robot_Init()
 */
void Init_LineFolower(void)
{
    LineFollower_Init(&line_follower);
    
    /* Optional: Customize configuration for your robot */
    LineFollower_Config_t custom_config = {
        .base_speed = 1300,          /* Adjust for your track speed */
        .max_steering = 650,         /* Increase for tighter turns, decrease for stability */
        .line_threshold = 2000,      /* Adjust based on your lighting and track color */
        .kp_steering = 1.1f,         /* Main responsiveness - higher = snappier */
        .ki_steering = 0.05f,        /* Drift correction - usually low */
        .kd_steering = 0.9f,         /* Oscillation damping - good for smooth follow */
        .line_lost_timeout_ms = 250  /* How long before giving up */
    };
    
    /* Uncomment to use custom config:
    LineFollower_SetConfig(&line_follower, &custom_config);
    */
    
    printf("\n=== LINE FOLLOWER READY ===\n");
    printf("Start: Press button or send 'S' command\n\n");
}

/**
 * @brief Main control loop - call at 200 Hz (5ms)
 * 
 * This is the snippet to integrate into your existing Robot_RunLoop()
 */
void LineFollower_ControlLoop(Mecanum_Chassis_t *chassis, uint32_t time_ms)
{
    /* Read sensors */
    extern ADC_HandleTypeDef hadc1;
    extern ADC_HandleTypeDef hadc2;
    extern ADC_HandleTypeDef hadc3;
    extern ADC_HandleTypeDef hadc4;
    
    QTR_Poll(&hadc1, &hadc2, &hadc3, &hadc4);
    
    /* Update line follower state machine */
    LineFollower_Update(&line_follower, time_ms);
    
    /* Get motor speeds from line follower */
    int32_t left_motor_speed;
    int32_t right_motor_speed;
    LineFollower_GetMotorSpeeds(&line_follower, &left_motor_speed, &right_motor_speed);
    
    /* Send to chassis: For mecanum/skid steer robots
       Both front-left and rear-left get left_motor_speed
       Both front-right and rear-right get right_motor_speed
    */
    if (chassis != NULL) {
        Motor_SetSpeed(&chassis->fl, left_motor_speed);
        Motor_SetSpeed(&chassis->rl, left_motor_speed);
        Motor_SetSpeed(&chassis->fr, right_motor_speed);
        Motor_SetSpeed(&chassis->rr, right_motor_speed);
    }
    
    /* Optional telemetry at 10 Hz */
    static uint32_t last_tele = 0;
    if (time_ms - last_tele >= 100) {
        last_tele = time_ms;
        
        extern uint16_t qtr_front[8];
        int32_t line_pos = QTR_GetFrontLinePosition();
        bool line_detected = LineFollower_IsLineDetected(&line_follower);
        
        printf("\r[LF] Left:%4ld Right:%4ld | Pos:%6ld | Detected:%s | State:%d",
               left_motor_speed, right_motor_speed, line_pos,
               line_detected ? "YES" : "NO ",
               (int)LineFollower_GetState(&line_follower));
        fflush(stdout);
    }
}

/**
 * @brief Command the line follower from UART or button
 * Call with: 'S' to start, 'X' to stop
 */
void LineFollower_Command(char cmd)
{
    switch (cmd) {
        case 'S':
        case 's':
            LineFollower_Start(&line_follower);
            break;
        
        case 'X':
        case 'x':
            LineFollower_Stop(&line_follower);
            break;
        
        case 'R':
        case 'r':
            LineFollower_Reset(&line_follower);
            printf("[LF] Reset complete\r\n");
            break;
        
        case 'C':
        case 'c': {
            /* Print current configuration */
            printf("\r\n=== LINE FOLLOWER CONFIG ===\r\n");
            printf("Base Speed:     %ld PWM\r\n", line_follower.config.base_speed);
            printf("Max Steering:   %ld PWM\r\n", line_follower.config.max_steering);
            printf("Kp (P-gain):    %.3f\r\n", (double)line_follower.config.kp_steering);
            printf("Ki (I-gain):    %.3f\r\n", (double)line_follower.config.ki_steering);
            printf("Kd (D-gain):    %.3f\r\n", (double)line_follower.config.kd_steering);
            printf("Line Threshold: %ld\r\n", line_follower.config.line_threshold);
            printf("State:          %d\r\n", (int)line_follower.state);
            printf("===========================\r\n");
            break;
        }
        
        case '+': {
            /* Increase base speed */
            line_follower.config.base_speed += 50;
            if (line_follower.config.base_speed > 1800)
                line_follower.config.base_speed = 1800;
            printf("\n[LF] Speed increased to %ld\r\n", line_follower.config.base_speed);
            break;
        }
        
        case '-': {
            /* Decrease base speed */
            line_follower.config.base_speed -= 50;
            if (line_follower.config.base_speed < 400)
                line_follower.config.base_speed = 400;
            printf("\n[LF] Speed decreased to %ld\r\n", line_follower.config.base_speed);
            break;
        }
        
        default:
            break;
    }
}
