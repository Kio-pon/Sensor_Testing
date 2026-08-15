#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <stdint.h>
#include <stdbool.h>
#include "pid.h"

/**
 * @brief Line Follower State Machine
 */
typedef enum {
    LF_IDLE,           /* Waiting for start command */
    LF_RUNNING,        /* Following the line */
    LF_LINE_LOST,      /* Lost the line */
    LF_FINISHED        /* Completed the course */
} LineFollower_State_t;

/**
 * @brief Line Follower Configuration
 */
typedef struct {
    int32_t base_speed;        /* Base forward speed (PWM units, ~1200-1500) */
    int32_t max_steering;      /* Maximum steering correction magnitude */
    int32_t line_threshold;    /* ADC threshold for line detection (typically 2000) */
    float kp_steering;         /* Proportional gain for steering PID */
    float ki_steering;         /* Integral gain for steering PID */
    float kd_steering;         /* Derivative gain for steering PID */
    uint32_t line_lost_timeout_ms; /* How long before line is considered lost (ms) */
} LineFollower_Config_t;

/**
 * @brief Line Follower Instance
 */
typedef struct {
    LineFollower_State_t state;
    LineFollower_Config_t config;
    PID_t steering_pid;
    
    int32_t last_line_position;
    uint32_t last_line_seen_time;
    bool line_detected;
    
    int32_t left_motor_speed;  /* Actual left motor PWM command */
    int32_t right_motor_speed; /* Actual right motor PWM command */
} LineFollower_t;

/**
 * @brief Initialize the line follower with default configuration
 */
void LineFollower_Init(LineFollower_t *lf);

/**
 * @brief Set custom configuration
 */
void LineFollower_SetConfig(LineFollower_t *lf, const LineFollower_Config_t *config);

/**
 * @brief Start line following
 */
void LineFollower_Start(LineFollower_t *lf);

/**
 * @brief Stop line following (coast to stop)
 */
void LineFollower_Stop(LineFollower_t *lf);

/**
 * @brief Main line following update (call at 200 Hz from control loop)
 * Reads front QTR, calculates steering, updates motor speeds
 */
void LineFollower_Update(LineFollower_t *lf, uint32_t time_ms);

/**
 * @brief Get the current motor speeds to send to chassis
 */
void LineFollower_GetMotorSpeeds(LineFollower_t *lf, int32_t *left_speed, int32_t *right_speed);

/**
 * @brief Get current state
 */
LineFollower_State_t LineFollower_GetState(LineFollower_t *lf);

/**
 * @brief Check if line is currently detected
 */
bool LineFollower_IsLineDetected(LineFollower_t *lf);

/**
 * @brief Get the current steering correction value
 */
int32_t LineFollower_GetSteeringCorrection(LineFollower_t *lf);

/**
 * @brief Reset any accumulated errors (for restarting on same track)
 */
void LineFollower_Reset(LineFollower_t *lf);

#endif /* LINE_FOLLOWER_H */
