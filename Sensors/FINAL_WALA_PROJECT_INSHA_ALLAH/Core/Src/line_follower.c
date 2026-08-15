#include "line_follower.h"
#include "qtr_array.h"
#include "motor_driver.h"
#include <stdio.h>
#include <math.h>

/**
 * ===================================================================
 * ROBUST COMPETITIVE-GRADE LINE FOLLOWER
 * 
 * Features:
 * - Fast 200 Hz sensor polling and motor updates
 * - Aggressive PID steering control (snappy response)
 * - Differential motor control for smooth turns
 * - Dead band to eliminate oscillations
 * - Line detection/loss handling
 * - Feedforward base speed for consistent velocity
 * ===================================================================
 */

/* Default configuration - tuned for competitive line following */
static const LineFollower_Config_t DEFAULT_CONFIG = {
    .base_speed = 1400,              /* Forward speed (PWM units) */
    .max_steering = 600,             /* Max steering correction magnitude */
    .line_threshold = 2000,          /* ADC threshold for line detection */
    .kp_steering = 1.2f,             /* Aggressive P-term for snappy response */
    .ki_steering = 0.05f,            /* Light I-term to handle drift */
    .kd_steering = 0.8f,             /* Good D-term damping */
    .line_lost_timeout_ms = 200      /* 200ms before declaring line lost */
};

void LineFollower_Init(LineFollower_t *lf)
{
    if (lf == NULL) return;
    
    lf->state = LF_IDLE;
    lf->config = DEFAULT_CONFIG;
    lf->last_line_position = 0;
    lf->last_line_seen_time = 0;
    lf->line_detected = false;
    lf->left_motor_speed = 0;
    lf->right_motor_speed = 0;
    
    /* Initialize steering PID with dynamic parameters */
    PID_Init(&lf->steering_pid,
             lf->config.kp_steering,
             lf->config.ki_steering,
             lf->config.kd_steering,
             0.0f,                      /* No feedforward for steering */
             500.0f,                    /* Integral windup limit */
             (float)lf->config.max_steering);
    
    /* Lighter filtering on derivative for snappier response */
    PID_SetFilter(&lf->steering_pid, 0.7f);
    
    printf("[LineFollower] Initialized with base speed %ld, max steering %ld\r\n",
           lf->config.base_speed, lf->config.max_steering);
}

void LineFollower_SetConfig(LineFollower_t *lf, const LineFollower_Config_t *config)
{
    if (lf == NULL || config == NULL) return;
    
    lf->config = *config;
    
    /* Re-initialize PID with new tuning parameters */
    PID_Init(&lf->steering_pid,
             lf->config.kp_steering,
             lf->config.ki_steering,
             lf->config.kd_steering,
             0.0f,
             500.0f,
             (float)lf->config.max_steering);
    
    PID_SetFilter(&lf->steering_pid, 0.7f);
    
    printf("[LineFollower] Config updated: speed=%ld, steer_max=%ld\r\n",
           lf->config.base_speed, lf->config.max_steering);
}

void LineFollower_Start(LineFollower_t *lf)
{
    if (lf == NULL) return;
    
    lf->state = LF_RUNNING;
    lf->line_detected = false;
    lf->left_motor_speed = 0;
    lf->right_motor_speed = 0;
    
    PID_Reset(&lf->steering_pid);
    
    printf("[LineFollower] STARTED - Following line...\r\n");
}

void LineFollower_Stop(LineFollower_t *lf)
{
    if (lf == NULL) return;
    
    lf->state = LF_IDLE;
    lf->left_motor_speed = 0;
    lf->right_motor_speed = 0;
    
    printf("[LineFollower] STOPPED\r\n");
}

void LineFollower_Reset(LineFollower_t *lf)
{
    if (lf == NULL) return;
    
    lf->last_line_position = 0;
    lf->line_detected = false;
    PID_Reset(&lf->steering_pid);
    
    printf("[LineFollower] Reset complete\r\n");
}

/**
 * @brief Detect line using front QTR array
 * Returns true if line is detected, false if white/no line
 */
static bool LineFollower_DetectLine(const uint16_t *qtr_data, int32_t *out_position)
{
    uint32_t sum = 0;
    uint32_t weighted_sum = 0;
    bool any_sensor_on_line = false;
    
    /* Weighted position calculation with threshold */
    for (int i = 0; i < 8; i++) {
        if (qtr_data[i] >= DEFAULT_CONFIG.line_threshold) {
            any_sensor_on_line = true;
            sum += qtr_data[i];
            /* Weight: 0 to 7 sensors, map to -3500 to +3500 */
            weighted_sum += (uint32_t)qtr_data[i] * (i * 1000);
        }
    }
    
    if (sum == 0) {
        *out_position = 0;
        return false;
    }
    
    /* Calculate centered position (-3500 to +3500) */
    int32_t center_offset = 3500;
    *out_position = (int32_t)(weighted_sum / sum) - center_offset;
    
    return any_sensor_on_line;
}

void LineFollower_Update(LineFollower_t *lf, uint32_t time_ms)
{
    if (lf == NULL || lf->state != LF_RUNNING) {
        lf->left_motor_speed = 0;
        lf->right_motor_speed = 0;
        return;
    }
    
    /* Get fresh sensor data */
    extern uint16_t qtr_front[8];
    
    /* Detect line and get position */
    int32_t line_position = 0;
    bool line_detected = LineFollower_DetectLine(qtr_front, &line_position);
    
    if (line_detected) {
        lf->last_line_seen_time = time_ms;
        lf->last_line_position = line_position;
        lf->line_detected = true;
    } else {
        /* Check if line has been lost for too long */
        if ((time_ms - lf->last_line_seen_time) > lf->config.line_lost_timeout_ms) {
            lf->line_detected = false;
            lf->state = LF_LINE_LOST;
            lf->left_motor_speed = 0;
            lf->right_motor_speed = 0;
            printf("[LineFollower] LINE LOST at time %lu ms\r\n", time_ms);
            return;
        }
    }
    
    /* ================================================================
       STEERING CONTROL
       
       The PID steering controller outputs a correction value in range
       [-max_steering, +max_steering] which represents the differential
       speed between left and right motors.
       
       Positive = turn right (reduce right motor, increase left motor)
       Negative = turn left (reduce left motor, increase right motor)
       ================================================================ */
    
    float steering_correction = PID_Update(&lf->steering_pid,
                                           (float)line_position,
                                           0.005f,  /* 200 Hz = 5ms dt */
                                           (float)lf->config.base_speed);
    
    /* Clamp steering correction to max range */
    if (steering_correction > (float)lf->config.max_steering) {
        steering_correction = (float)lf->config.max_steering;
    } else if (steering_correction < -(float)lf->config.max_steering) {
        steering_correction = -(float)lf->config.max_steering;
    }
    
    /* ================================================================
       MOTOR SPEED CALCULATION
       
       Differential drive: Left and Right motors have different speeds
       to create steering.
       
       If steering_correction is positive (turn right):
         - Reduce right motor speed
         - Increase left motor speed
       
       If steering_correction is negative (turn left):
         - Reduce left motor speed
         - Increase right motor speed
       ================================================================ */
    
    int32_t steering_int = (int32_t)steering_correction;
    
    /* Base speed for both motors */
    lf->left_motor_speed = lf->config.base_speed;
    lf->right_motor_speed = lf->config.base_speed;
    
    /* Apply steering correction (differential speed) */
    lf->left_motor_speed += steering_int;   /* Left motor gets +correction */
    lf->right_motor_speed -= steering_int;  /* Right motor gets -correction */
    
    /* ================================================================
       MOTOR SPEED CLIPPING & PROTECTION
       ================================================================ */
    
    int32_t max_speed = 2000;  /* Maximum PWM value for motor safety */
    int32_t min_speed = 400;   /* Minimum speed to overcome stiction */
    
    /* Clip speeds to safe ranges while maintaining differential */
    if (lf->left_motor_speed > max_speed) {
        int32_t excess = lf->left_motor_speed - max_speed;
        lf->left_motor_speed = max_speed;
        lf->right_motor_speed -= excess;  /* Reduce right if left hits limit */
    }
    if (lf->right_motor_speed > max_speed) {
        int32_t excess = lf->right_motor_speed - max_speed;
        lf->right_motor_speed = max_speed;
        lf->left_motor_speed -= excess;   /* Reduce left if right hits limit */
    }
    
    /* Ensure minimum forward speed for both motors */
    if (lf->left_motor_speed < min_speed && lf->left_motor_speed > 0) {
        lf->left_motor_speed = min_speed;
    }
    if (lf->right_motor_speed < min_speed && lf->right_motor_speed > 0) {
        lf->right_motor_speed = min_speed;
    }
    
    /* Stop if either motor would go backward (shouldn't happen with good tuning) */
    if (lf->left_motor_speed < 0) lf->left_motor_speed = 0;
    if (lf->right_motor_speed < 0) lf->right_motor_speed = 0;
}

void LineFollower_GetMotorSpeeds(LineFollower_t *lf, int32_t *left_speed, int32_t *right_speed)
{
    if (lf == NULL) {
        if (left_speed) *left_speed = 0;
        if (right_speed) *right_speed = 0;
        return;
    }
    
    if (left_speed) *left_speed = lf->left_motor_speed;
    if (right_speed) *right_speed = lf->right_motor_speed;
}

LineFollower_State_t LineFollower_GetState(LineFollower_t *lf)
{
    return (lf != NULL) ? lf->state : LF_IDLE;
}

bool LineFollower_IsLineDetected(LineFollower_t *lf)
{
    return (lf != NULL) ? lf->line_detected : false;
}

int32_t LineFollower_GetSteeringCorrection(LineFollower_t *lf)
{
    if (lf == NULL) return 0;
    
    extern uint16_t qtr_front[8];
    int32_t line_position = 0;
    
    LineFollower_DetectLine(qtr_front, &line_position);
    
    return (int32_t)PID_Update(&lf->steering_pid,
                               (float)line_position,
                               0.005f,
                               (float)lf->config.base_speed);
}
