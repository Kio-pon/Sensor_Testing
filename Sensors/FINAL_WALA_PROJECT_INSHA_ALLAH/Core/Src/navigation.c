#include "navigation.h"
#include "robot_config.h"
#include "robot_core.h"
#include "pid.h"
#include "qtr_array.h"
#include "sharp_ir.h"
#include "tcs34725.h"
#include "encoders.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;

#define ABS(x) ((x) < 0 ? -(x) : (x))

NavState_t nav_state = NAV_IDLE;
static uint8_t sequence_step = 0;
static bool decel_started = false;

/* Targets */
int32_t target_ticks = 0;
static int32_t target_sharp_cm = 0;
static float   target_yaw = 0.0f;
static int32_t current_speed = 0;
static int32_t current_vx = 0;
static int32_t current_vy = 0;
int32_t start_ticks = 0;
static int32_t min_strafe_ticks = 0;

/* Color Check Variables */
static char uart_rx_buf[16];
static uint8_t uart_rx_idx = 0;
static uint32_t last_color_print_time = 0;

/* Absolute Target Coordinate */
static float global_target_x = 0.0f;
static float global_target_y = 0.0f;

/* Line Following PIDs */
static PID_t line_pid;
static PID_t strafe_line_pid;

/* External variables from robot_core.c */
extern volatile int32_t enc1_count;
extern volatile int32_t enc2_count;
extern volatile int32_t enc3_count;
extern volatile int32_t enc4_count;
extern float gyro_yaw_deg;

/* Helper to get average absolute distance travelled by wheels */
static int32_t Get_Avg_Encoder_Ticks(void) {
    int32_t e1 = ABS(enc1_count);
    int32_t e2 = ABS(enc2_count);
    int32_t e3 = ABS(enc3_count);
    int32_t e4 = ABS(enc4_count);
    return (e1 + e2 + e3 + e4) / 4;
}

void Nav_Init(void) {
    /* Initialize Line Following PIDs 
       Tuning: Kp, Ki, Kd, kv, min_out, max_out */
    PID_Init(&line_pid, 0.4f, 0.0f, 0.02f, 0.0f, -1500.0f, 1500.0f);
    PID_SetFilter(&line_pid, 0.3f); // Heavy filtering for high speed stability
    
    PID_Init(&strafe_line_pid, 1.2f, 0.0f, 0.15f, 0.0f, -1500.0f, 1500.0f);
    PID_SetFilter(&strafe_line_pid, 0.3f);
    
    nav_state = NAV_IDLE;
    sequence_step = 0;
}

bool Nav_IsBusy(void) {
    return (nav_state != NAV_IDLE);
}

void Nav_Stop(void) {
    nav_state = NAV_IDLE;
    robot_vx = 0;
    robot_vy = 0;
    target_heading = gyro_yaw_deg; // Lock heading to where we currently are
}

void Nav_FollowLineForDistance(float cm, int32_t speed) {
    Encoders_ResetAll();
    start_ticks = 0;
    target_ticks = (int32_t)(cm * TICKS_PER_CM);
    current_speed = speed;
    decel_started = false;
    target_heading = 0.0f; // Lock target heading to 0.0f (the start orientation)
    nav_state = NAV_FOLLOW_LINE_DIST;
}

void Nav_StartMove(float cm, int32_t vx, int32_t vy) {
    Encoders_ResetAll();
    start_ticks = 0;
    target_ticks = (int32_t)(cm * TICKS_PER_CM);
    current_vx = vx;
    current_vy = vy;
    nav_state = NAV_MOVE_DIST;
}

void Nav_StrafeAlongLineUntilJunction(int32_t speed_vx, float min_cm) {
    Encoders_ResetAll();
    start_ticks = 0;
    min_strafe_ticks = Strafe_CM_To_Ticks(min_cm);
    target_ticks = 0; // Disable distance check
    current_vx = speed_vx;
    current_vy = 0; // Handled by PID
    nav_state = NAV_STRAFE_LOCKED;
}

void Nav_StrafeAlongLineForDistance(float cm, int32_t speed_vx) {
    Encoders_ResetAll();
    start_ticks = 0;
    target_ticks = (int32_t)(Strafe_CM_To_Ticks(cm) * STRAFE_TICKS_SCALE);
    current_vx = speed_vx;
    current_vy = 0; // Handled by PID
    nav_state = NAV_STRAFE_LOCKED;
}

void Nav_StartColorCheck(void) {
    uart_rx_idx = 0;
    memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
    nav_state = NAV_CHECK_COLOR;
}

/* NERC Optimization: Gyro Snap-to-Grid */
static void Nav_SnapToGrid(void) {
    /* Snap the current absolute target_heading to the nearest 90 degrees */
    float snapped = roundf(target_heading / 90.0f) * 90.0f;
    target_heading = snapped;
    
    /* Wrap to [-180, 180] */
    while (target_heading > 180.0f) target_heading -= 360.0f;
    while (target_heading < -180.0f) target_heading += 360.0f;
}

void Nav_FollowLineUntilDistance(float target_cm, int32_t speed) {
    target_sharp_cm = (int32_t)target_cm;
    current_speed = speed;
    nav_state = NAV_FOLLOW_LINE_KEEP_DIST;
}

void Nav_GoToCoordinate(float target_x_mm, float target_y_mm, int32_t speed) {
    global_target_x = target_x_mm;
    global_target_y = target_y_mm;
    current_speed = speed;
    nav_state = NAV_GOTO_COORD;
}

/* ============================================================
   SEQUENCE ENGINEote: 'target_heading' is the global variable in robot_core.c that the gyro PID holds */
void Nav_StartTurn(float degrees, int32_t speed) {
    /* Note: 'target_heading' is the global variable in robot_core.c that the gyro PID holds */
    target_heading += degrees; 
    
    // Wrap to [-180, 180]
    while (target_heading > 180.0f) target_heading -= 360.0f;
    while (target_heading < -180.0f) target_heading += 360.0f;
    
    target_yaw = target_heading;
    nav_state = NAV_TURN;
}

/* Hardcoded Sequence Logic */
void Nav_RunSequence(void) {
    if (Nav_IsBusy()) return;
    
    switch (sequence_step) {
        case 0:
            /* Step 1: Move 103cm forward (+12cm decel compensation) and follow the line */
            Nav_FollowLineForDistance(TARGET_STRAIGHT_CM + DECEL_COMPENSATION_CM, 1500);
            sequence_step++;
            break;
            
        case 1:
            /* Step 2: Strafe RIGHT 2 feet (60.96cm) at speed 2500 */
            Nav_StrafeAlongLineForDistance(60.96f, 2500);
            target_heading = STRAFE_CORRECTION_DEG; // Add clockwise correction
            sequence_step++;
            break;

        case 2:
            /* Step 3: Move BACKWARD 2 feet (60.96cm) using line following and encoder */
            Nav_FollowLineForDistance(60.96f, -1500);
            sequence_step++;
            break;

        case 3:
            /* Step 4: Stop */
            Nav_Stop();
            break;
    }
}

/* The actual state machine pumped every 5ms */
bool Nav_Update(void) {
    if (nav_state == NAV_IDLE) return false;

    int32_t omega = 0;
    int32_t line_error = 0;
    int32_t ticks_travelled = 0;
    int32_t sharp_dist = 0;

    switch (nav_state) {
        case NAV_MOVE_DIST:
            ticks_travelled = Get_Avg_Encoder_Ticks() - start_ticks;
            if (ticks_travelled >= target_ticks) {
                Nav_Stop();
                return false;
            }
            robot_vx = current_vx;
            robot_vy = current_vy;
            break;

        case NAV_FOLLOW_LINE_DIST:
            ticks_travelled = Get_Avg_Encoder_Ticks() - start_ticks;
            
            /* Have we reached the target distance? */
            if (ticks_travelled >= target_ticks) {
                Nav_Stop();
                return false;
            }

            int32_t ticks_remaining = target_ticks - ticks_travelled;
            int32_t deceleration_start_ticks = (int32_t)(20.0f * TICKS_PER_CM); // Decelerate over the last 20cm
            int32_t speed = current_speed;
            bool in_decel_phase = false;

            if (ticks_remaining < deceleration_start_ticks && ticks_remaining > 0) {
                in_decel_phase = true;
                if (!decel_started) {
                    decel_started = true;
                    // Keep target_heading at the start direction (0.0f) instead of locking to current yaw
                }
                float ratio = (float)ticks_remaining / deceleration_start_ticks;
                if (ratio < 0.0f) ratio = 0.0f;
                if (ratio > 1.0f) ratio = 1.0f;
                
                int32_t base_decel_speed = (current_speed >= 0) ? 600 : -600;
                speed = base_decel_speed + (int32_t)((current_speed - base_decel_speed) * ratio);
            }

            /* PID Line Following (only active when not in gyro-locked decel phase) */
            if (!in_decel_phase) {
                line_error = QTR_GetFrontLinePosition(); // 0 is perfectly centered
                float error_input = (current_speed >= 0) ? (float)(-line_error) : (float)(line_error);
                omega = (int32_t)PID_Update(&line_pid, error_input, 0.005f, 0.0f);
            } else {
                omega = 0; // Handled by Gyro PID in robot_core.c
            }
            
            robot_vy = speed;
            robot_vx = 0;
            break;

        case NAV_STRAFE_LOCKED:
            ticks_travelled = Get_Avg_Encoder_Ticks() - start_ticks;
            
            /* 1. PID Line Following using average of Left and Right QTR arrays */
            line_error = (QTR_GetLeftLinePosition() + QTR_GetRightLinePosition()) / 2;
            int32_t vy_correction = (int32_t)PID_Update(&strafe_line_pid, (float)(-line_error), 0.005f, 0.0f);
            
            /* 2. Gyro locks heading natively in robot_core.c */
            /* Implement deceleration as we approach expected junction */
            int32_t speed_vx = current_vx;
            if (target_ticks == 0) { // Junction mode
                float cm_travelled = (float)ticks_travelled / TICKS_PER_CM;
                if (cm_travelled > STRAFE_DECEL_START_CM) {
                    float ratio = (EXPECTED_JUNCTION_CM - cm_travelled) / (EXPECTED_JUNCTION_CM - STRAFE_DECEL_START_CM);
                    if (ratio < 0.0f) ratio = 0.0f;
                    if (ratio > 1.0f) ratio = 1.0f;
                    speed_vx = STRAFE_MIN_SPEED + (int32_t)((current_vx - STRAFE_MIN_SPEED) * ratio);
                }
            }
            robot_vx = speed_vx;
            robot_vy = vy_correction; // Keeps the robot perfectly on the horizontal line
            
            /* 3. Check for completion condition */
            if (target_ticks > 0) {
                // Distance mode
                if (ticks_travelled >= target_ticks) {
                    Nav_Stop();
                    return false;
                }
            } else {
                // Junction mode
                if (ticks_travelled > min_strafe_ticks) {
                    /* STRICT JUNCTION: Front middle, Left middle, and Right working outer pins see black */
                    bool front_cross = (qtr_front[3] > 3000 || qtr_front[4] > 3000);
                    bool left_cross  = (qtr_left[2] > 3000  || qtr_left[3] > 3000);
                    bool right_cross = (qtr_right[0] > 3000 || qtr_right[1] > 3000 || qtr_right[4] > 3000 || qtr_right[5] > 3000);
                    
                    if (front_cross && left_cross && right_cross) {
                        Nav_Stop();
                        return false;
                    }
                }
            }
            break;

        case NAV_CHECK_COLOR:
            /* Stop moving */
            robot_vx = 0;
            robot_vy = 0;
            omega = 0;
            
            /* 1. Poll Color Sensor every 500ms so we don't flood the UART */
            if (HAL_GetTick() - last_color_print_time > 500) {
                TCS34725_RawData raw;
                if (TCS34725_ReadRaw(&hi2c1, &raw) == HAL_OK) {
                    DetectedColor color = TCS34725_ClassifyColor(&raw);
                    if (color == COLOR_RED) {
                        printf("RED\r\n");
                        last_color_print_time = HAL_GetTick();
                    } else if (color == COLOR_BLUE) {
                        printf("BLUE\r\n");
                        last_color_print_time = HAL_GetTick();
                    }
                }
            }
            
            /* 2. Non-blocking UART check for "done" */
            if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
                char c = (char)(huart1.Instance->RDR & 0xFF);
                if (c == '\n' || c == '\r' || c == 'e') { 
                    /* If we hit newline, or the 'e' in "done" */
                    if (uart_rx_idx < sizeof(uart_rx_buf) - 1) {
                        uart_rx_buf[uart_rx_idx++] = c;
                    }
                    if (strstr(uart_rx_buf, "done") != NULL) {
                        /* Received "done"! Proceed to next sequence step */
                        Nav_Stop();
                        return false; 
                    }
                    /* Reset buffer on newline to avoid overflow */
                    if (c == '\n' || c == '\r') {
                        uart_rx_idx = 0;
                        memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
                    }
                } else {
                    if (uart_rx_idx < sizeof(uart_rx_buf) - 1) {
                        uart_rx_buf[uart_rx_idx++] = c;
                    }
                }
            }
            break;

        case NAV_FOLLOW_LINE_KEEP_DIST:
            sharp_dist = (int32_t)Sharp_GetDistanceCM();
            
            /* If we see the wall (or object) at or below our target distance, STOP */
            if (sharp_dist > 0 && sharp_dist <= target_sharp_cm) {
                Nav_Stop();
                return false;
            }

            /* PID Line Following */
            line_error = QTR_GetFrontLinePosition();
            omega = (int32_t)PID_Update(&line_pid, (float)(-line_error), 0.005f, 0.0f);
            
            robot_vy = current_speed;
            robot_vx = 0;
            break;

        case NAV_GOTO_COORD:
            {
                /* 1. Calculate world error */
                float dx = global_target_x - global_x;
                float dy = global_target_y - global_y;
                float dist = sqrtf(dx*dx + dy*dy);
                
                /* 2. Check if arrived (within 10mm) */
                if (dist < 10.0f) {
                    Nav_Stop();
                    return false;
                }
                
                /* 3. Normalize world vector */
                float norm_dx = dx / dist;
                float norm_dy = dy / dist;
                
                /* 4. Rotate world vector into robot's local frame */
                float yaw_rad = gyro_yaw_deg * (M_PI / 180.0f);
                float cos_y = cosf(-yaw_rad); // negative because inverse rotation
                float sin_y = sinf(-yaw_rad);
                
                float local_dx = norm_dx * cos_y - norm_dy * sin_y;
                float local_dy = norm_dx * sin_y + norm_dy * cos_y;
                
                /* 5. Command velocities (Gyro handles rotation natively in robot_core!) */
                robot_vx = (int32_t)(local_dx * current_speed);
                robot_vy = (int32_t)(local_dy * current_speed);
            }
            break;

        case NAV_TURN:
            /* Let the gyro PID handle the turning in robot_core.c. We just wait for error to be small. */
            robot_vy = 0;
            robot_vx = 0;
            
            float error = target_yaw - gyro_yaw_deg;
            while (error > 180.0f) error -= 360.0f;
            while (error < -180.0f) error += 360.0f;
            
            if (fabs(error) < 2.0f) { // Within 2 degrees
                Nav_SnapToGrid(); // Instantly aligns our target heading to a perfect grid angle!
                Nav_Stop();
                return false;
            }
            break;
            
        case NAV_STOP:
        case NAV_IDLE:
        default:
            break;
    }
    
    /* Export line-following omega to the chassis if we are in a line-following state. */
    if ((nav_state == NAV_FOLLOW_LINE_DIST && !decel_started) || nav_state == NAV_FOLLOW_LINE_KEEP_DIST) {
        extern int32_t nav_omega_override;
        extern bool use_nav_omega;
        nav_omega_override = omega;
        use_nav_omega = true;
    } else {
        extern bool use_nav_omega;
        use_nav_omega = false;
    }

    return true;
}
