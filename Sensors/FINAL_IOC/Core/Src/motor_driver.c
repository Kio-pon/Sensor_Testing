#include "motor_driver.h"
#include "encoders.h"
#include <stddef.h>
#include <math.h>
#include <stdio.h>

void Motor_Init(TB6612_Motor_t *motor)
{
    if (motor == NULL) {
        printf("  [Motor_Init] ERROR: Motor pointer is NULL!\r\n");
        return;
    }
    if (motor->htim == NULL) {
        printf("  [Motor_Init] ERROR: Motor Timer handle is NULL!\r\n");
        return;
    }
    
    printf("  [Motor_Init] Starting PWM timer %p channel %lu...\r\n", (void*)motor->htim, motor->channel);
    HAL_StatusTypeDef status = HAL_TIM_PWM_Start(motor->htim, motor->channel);
    printf("  [Motor_Init] PWM timer started with status: %d\r\n", (int)status);
    
    // Ensure direction inputs are reset
    Motor_Stop(motor);
}

void Motor_SetSpeed(TB6612_Motor_t *motor, int32_t speed)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // Clamp speed limits
    if (speed > (int32_t)motor->max_pwm) speed = (int32_t)motor->max_pwm;
    if (speed < -(int32_t)motor->max_pwm) speed = -(int32_t)motor->max_pwm;
    
    // Apply custom scale to bypass stiction (50% to 100% PWM linear mapping)
    int32_t scaled_compare = 0;
    if (speed > 0) {
        // S_scaled = (50 * M + 50 * S) / 100
        scaled_compare = (50 * (int32_t)motor->max_pwm + 50 * speed) / 100;
        
        // Forward rotation
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint32_t)scaled_compare);
    } 
    else if (speed < 0) {
        // S_scaled = (50 * M + 50 * (-S)) / 100
        scaled_compare = (50 * (int32_t)motor->max_pwm + 50 * (-speed)) / 100;
        
        // Reverse rotation
        HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, (uint32_t)scaled_compare);
    } 
    else {
        // Stop
        Motor_Brake(motor);
    }
}

void Motor_Brake(TB6612_Motor_t *motor)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // Short Brake: Pull both IN1 and IN2 HIGH to short-circuit windings
    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, motor->max_pwm); // High braking duty
}

void Motor_Stop(TB6612_Motor_t *motor)
{
    if (motor == NULL || motor->htim == NULL) return;
    
    // Coast Stop: Pull both direction pins LOW
    HAL_GPIO_WritePin(motor->IN1_Port, motor->IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor->IN2_Port, motor->IN2_Pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(motor->htim, motor->channel, 0);
}

void Chassis_Init(Mecanum_Chassis_t *chassis)
{
    if (chassis == NULL) {
        printf("  [Chassis_Init] ERROR: Chassis pointer is NULL!\r\n");
        return;
    }
    
    printf("  [Chassis_Init] Initializing Front-Left Motor...\r\n");
    Motor_Init(&chassis->fl);
    printf("  [Chassis_Init] Initializing Front-Right Motor...\r\n");
    Motor_Init(&chassis->fr);
    printf("  [Chassis_Init] Initializing Rear-Left Motor...\r\n");
    Motor_Init(&chassis->rl);
    printf("  [Chassis_Init] Initializing Rear-Right Motor...\r\n");
    Motor_Init(&chassis->rr);
    
    // Pull standby HIGH to enable drivers initially
    printf("  [Chassis_Init] Setting Standby GPIO (STBY High)...\r\n");
    Chassis_SetStandby(chassis, 1);
    printf("  [Chassis_Init] Complete!\r\n");
}

void Chassis_SetStandby(Mecanum_Chassis_t *chassis, uint8_t state)
{
    if (chassis == NULL || chassis->STBY_Port == NULL) return;
    
    HAL_GPIO_WritePin(chassis->STBY_Port, chassis->STBY_Pin, 
                      state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Chassis_Drive(Mecanum_Chassis_t *chassis, int32_t vx, int32_t vy, int32_t omega)
{
    if (chassis == NULL) return;
    
    // Invert vy and omega to match the physical wiring and assembly direction
    vy = -vy;
    omega = -omega;
    
    // Mecanum Kinematic formulas
    int32_t fl_speed = vx + vy - omega;
    int32_t fr_speed = vx - vy + omega;
    int32_t rl_speed = vx - vy - omega;
    int32_t rr_speed = vx + vy + omega;
    
    Motor_SetSpeed(&chassis->fl, fl_speed);
    Motor_SetSpeed(&chassis->fr, fr_speed);
    Motor_SetSpeed(&chassis->rl, rl_speed);
    Motor_SetSpeed(&chassis->rr, rr_speed);
}

void Chassis_BrakeAll(Mecanum_Chassis_t *chassis)
{
    if (chassis == NULL) return;
    
    Motor_Brake(&chassis->fl);
    Motor_Brake(&chassis->fr);
    Motor_Brake(&chassis->rl);
    Motor_Brake(&chassis->rr);
}

void Chassis_CoastAll(Mecanum_Chassis_t *chassis)
{
    if (chassis == NULL) return;
    
    Motor_Stop(&chassis->fl);
    Motor_Stop(&chassis->fr);
    Motor_Stop(&chassis->rl);
    Motor_Stop(&chassis->rr);
}

static void Motor_SetSpeedsNormalized(Mecanum_Chassis_t *chassis, float lf, float rf, float lr, float rr)
{
    if (chassis == NULL) return;
    
    // Normalize if absolute value of any motor output exceeds 1.0
    float max_val = fabsf(lf);
    if (fabsf(rf) > max_val) max_val = fabsf(rf);
    if (fabsf(lr) > max_val) max_val = fabsf(lr);
    if (fabsf(rr) > max_val) max_val = fabsf(rr);
    
    if (max_val > 1.0f) {
        lf /= max_val;
        rf /= max_val;
        lr /= max_val;
        rr /= max_val;
    }
    
    int32_t fl_speed = (int32_t)(lf * chassis->fl.max_pwm);
    int32_t fr_speed = (int32_t)(rf * chassis->fr.max_pwm);
    int32_t rl_speed = (int32_t)(lr * chassis->rl.max_pwm);
    int32_t rr_speed = (int32_t)(rr * chassis->rr.max_pwm);
    
    Motor_SetSpeed(&chassis->fl, fl_speed);
    Motor_SetSpeed(&chassis->fr, fr_speed);
    Motor_SetSpeed(&chassis->rl, rl_speed);
    Motor_SetSpeed(&chassis->rr, rr_speed);
}

void Chassis_DriveForwardBackward(Mecanum_Chassis_t *chassis, int32_t speed, int32_t turn, uint32_t duration_ms)
{
    if (chassis == NULL) return;
    
    float p = (float)speed / chassis->fl.max_pwm;
    float t = (float)turn / chassis->fl.max_pwm;
    
    // Forward (Up)
    Motor_SetSpeedsNormalized(chassis, p + t, p - t, p + t, p - t);
    HAL_Delay(duration_ms);
    
    // Short Brake & Pause
    Chassis_BrakeAll(chassis);
    HAL_Delay(500);
    
    // Backward (Down)
    Motor_SetSpeedsNormalized(chassis, -p + t, -p - t, -p + t, -p - t);
    HAL_Delay(duration_ms);
    
    // Final Brake
    Chassis_BrakeAll(chassis);
}

void Chassis_DriveStrafe(Mecanum_Chassis_t *chassis, int32_t speed, int32_t turn, uint32_t duration_ms)
{
    if (chassis == NULL) return;
    
    float p = (float)speed / chassis->fl.max_pwm;
    float t = (float)turn / chassis->fl.max_pwm;
    
    // Strafe Right
    Motor_SetSpeedsNormalized(chassis, p + t, -p - t, -p + t, p - t);
    HAL_Delay(duration_ms);
    
    // Short Brake & Pause
    Chassis_BrakeAll(chassis);
    HAL_Delay(500);
    
    // Strafe Left
    Motor_SetSpeedsNormalized(chassis, -p + t, p - t, p + t, -p - t);
    HAL_Delay(duration_ms);
    
    // Final Brake
    Chassis_BrakeAll(chassis);
}

void Chassis_DriveDiagonals(Mecanum_Chassis_t *chassis, int32_t speed, int32_t turn, uint32_t duration_ms)
{
    if (chassis == NULL) return;
    
    float p = (float)speed / chassis->fl.max_pwm;
    float t = (float)turn / chassis->fl.max_pwm;
    
    // Diagonal Forward-Right
    Motor_SetSpeedsNormalized(chassis, p + t, -t, t, p - t);
    HAL_Delay(duration_ms);
    Chassis_BrakeAll(chassis);
    HAL_Delay(500);
    
    // Diagonal Backward-Left
    Motor_SetSpeedsNormalized(chassis, -p + t, -t, t, -p - t);
    HAL_Delay(duration_ms);
    Chassis_BrakeAll(chassis);
    HAL_Delay(500);
    
    // Diagonal Forward-Left
    Motor_SetSpeedsNormalized(chassis, t, p - t, p + t, -t);
    HAL_Delay(duration_ms);
    Chassis_BrakeAll(chassis);
    HAL_Delay(500);
    
    // Diagonal Backward-Right
    Motor_SetSpeedsNormalized(chassis, t, -p - t, -p + t, -t);
    HAL_Delay(duration_ms);
    Chassis_BrakeAll(chassis);
}

void Chassis_DriveArbitraryAngle(Mecanum_Chassis_t *chassis, int32_t speed, int32_t turn, float angle_deg, uint32_t duration_ms)
{
    if (chassis == NULL) return;
    
    float p = (float)speed / chassis->fl.max_pwm;
    float t = (float)turn / chassis->fl.max_pwm;
    
    // Convert angle to radians
    float theta = angle_deg * (3.1415926535f / 180.0f);
    
    // Offset by Pi/4
    float sin_val = sinf(theta - 3.1415926535f / 4.0f);
    float cos_val = cosf(theta - 3.1415926535f / 4.0f);
    
    // Find the maximum component to scale it properly (maintaining maximum power)
    float max_val = fabsf(sin_val);
    if (fabsf(cos_val) > max_val) max_val = fabsf(cos_val);
    if (max_val < 0.0001f) max_val = 1.0f; // avoid division by zero
    
    float lf = p * (cos_val / max_val) + t;
    float rf = p * (sin_val / max_val) - t;
    float lr = p * (sin_val / max_val) + t;
    float rr = p * (cos_val / max_val) - t;
    
    Motor_SetSpeedsNormalized(chassis, lf, rf, lr, rr);
    HAL_Delay(duration_ms);
    
    Chassis_BrakeAll(chassis);
}

void Chassis_DriveCircleDiameter(Mecanum_Chassis_t *chassis, int32_t speed, int32_t turn, float diameter_angle_deg, uint32_t duration_ms)
{
    if (chassis == NULL) return;
    
    // Drive out along the diameter angle
    Chassis_DriveArbitraryAngle(chassis, speed, turn, diameter_angle_deg, duration_ms);
    
    // Short Brake & Pause
    Chassis_BrakeAll(chassis);
    HAL_Delay(500);
    
    // Drive back in the opposite direction (diameter_angle_deg + 180 degrees)
    float opposite_angle = diameter_angle_deg + 180.0f;
    if (opposite_angle >= 360.0f) opposite_angle -= 360.0f;
    Chassis_DriveArbitraryAngle(chassis, speed, turn, opposite_angle, duration_ms);
    
    // Final Brake
    Chassis_BrakeAll(chassis);
}

void Chassis_RotateInPlace(Mecanum_Chassis_t *chassis, int32_t turn, uint32_t duration_ms)
{
    if (chassis == NULL) return;
    
    float t = (float)turn / chassis->fl.max_pwm;
    
    // Rotate Clockwise (Left side forward, Right side backward)
    Motor_SetSpeedsNormalized(chassis, t, -t, t, -t);
    HAL_Delay(duration_ms);
    
    // Short Brake & Pause
    Chassis_BrakeAll(chassis);
    HAL_Delay(500);
    
    // Rotate Counter-Clockwise (Left side backward, Right side forward)
    Motor_SetSpeedsNormalized(chassis, -t, t, -t, t);
    HAL_Delay(duration_ms);
    
    // Final Brake
    Chassis_BrakeAll(chassis);
}

void Chassis_DriveForwardBackwardRamped(Mecanum_Chassis_t *chassis, int32_t target_speed, int32_t turn, uint32_t duration_ms, uint32_t ramp_time_ms)
{
    if (chassis == NULL || ramp_time_ms == 0) return;
    
    uint32_t steps = 20; // 20 steps for smooth ramping
    uint32_t step_delay = ramp_time_ms / steps;
    if (step_delay == 0) step_delay = 1;
    
    // --- FORWARD SEQUENCE ---
    // 1. Speed Up (Ramp up forward)
    for (uint32_t i = 1; i <= steps; i++) {
        int32_t current_speed = (target_speed * i) / steps;
        float p = (float)current_speed / chassis->fl.max_pwm;
        float t = (float)turn / chassis->fl.max_pwm;
        Motor_SetSpeedsNormalized(chassis, p + t, p - t, p + t, p - t);
        HAL_Delay(step_delay);
    }
    
    // 2. Drive at Target Speed
    float p_target = (float)target_speed / chassis->fl.max_pwm;
    float t_target = (float)turn / chassis->fl.max_pwm;
    Motor_SetSpeedsNormalized(chassis, p_target + t_target, p_target - t_target, p_target + t_target, p_target - t_target);
    HAL_Delay(duration_ms);
    
    // 3. Speed Down (Ramp down forward)
    for (uint32_t i = steps; i >= 1; i--) {
        int32_t current_speed = (target_speed * i) / steps;
        float p = (float)current_speed / chassis->fl.max_pwm;
        float t = (float)turn / chassis->fl.max_pwm;
        Motor_SetSpeedsNormalized(chassis, p + t, p - t, p + t, p - t);
        HAL_Delay(step_delay);
    }
    
    // Short Brake & Pause
    Chassis_BrakeAll(chassis);
    HAL_Delay(500);
    
    // --- BACKWARD SEQUENCE ---
    // 1. Speed Up (Ramp up backward)
    for (uint32_t i = 1; i <= steps; i++) {
        int32_t current_speed = (target_speed * i) / steps;
        float p = (float)current_speed / chassis->fl.max_pwm;
        float t = (float)turn / chassis->fl.max_pwm;
        Motor_SetSpeedsNormalized(chassis, -p + t, -p - t, -p + t, -p - t);
        HAL_Delay(step_delay);
    }
    
    // 2. Drive at Target Speed Backward
    Motor_SetSpeedsNormalized(chassis, -p_target + t_target, -p_target - t_target, -p_target + t_target, -p_target - t_target);
    HAL_Delay(duration_ms);
    
    // 3. Speed Down (Ramp down backward)
    for (uint32_t i = steps; i >= 1; i--) {
        int32_t current_speed = (target_speed * i) / steps;
        float p = (float)current_speed / chassis->fl.max_pwm;
        float t = (float)turn / chassis->fl.max_pwm;
        Motor_SetSpeedsNormalized(chassis, -p + t, -p - t, -p + t, -p - t);
        HAL_Delay(step_delay);
    }
    
    // Final Brake
    Chassis_BrakeAll(chassis);
}

void Chassis_DriveOptimizedDPad(Mecanum_Chassis_t *chassis, int x, int y, float turn)
{
    if (chassis == NULL) return;
    
    float leftFront, rightFront, leftRear, rightRear;
    
    // Hardcode logic for D-Pad style efficiency
    if (y > 0) { 
        // Up (Forward)
        leftFront  = 1.0f + turn; rightFront = 1.0f - turn; leftRear   = 1.0f + turn; rightRear  = 1.0f - turn;
    } 
    else if (y < 0) { 
        // Down (Backward)
        leftFront  = -1.0f + turn; rightFront = -1.0f - turn; leftRear   = -1.0f + turn; rightRear  = -1.0f - turn;
    } 
    else if (x > 0) {
        // Right
        leftFront  = 1.0f + turn; rightFront = -1.0f - turn; leftRear   = -1.0f + turn; rightRear  = 1.0f - turn;
    }
    else if (x < 0) {
        // Left
        leftFront  = -1.0f + turn; rightFront = 1.0f - turn; leftRear   = 1.0f + turn; rightRear  = -1.0f - turn;
    }
    else {
        // Just turning or Stop
        leftFront  = turn; rightFront = -turn; leftRear   = turn; rightRear  = -turn;
    }

    // Conditional Normalization
    float max = fmaxf(fmaxf(fabsf(leftFront), fabsf(rightFront)), fmaxf(fabsf(leftRear), fabsf(rightRear)));
    if (max > 1.0f) {
        leftFront  /= max;
        rightFront /= max;
        leftRear   /= max;
        rightRear  /= max;
    }
    
    // Default to 50% max speed for the test sequence
    float speed_scale = 0.5f; 

    Motor_SetSpeed(&chassis->fl, (int32_t)(leftFront * chassis->fl.max_pwm * speed_scale));
    Motor_SetSpeed(&chassis->fr, (int32_t)(rightFront * chassis->fr.max_pwm * speed_scale));
    Motor_SetSpeed(&chassis->rl, (int32_t)(leftRear * chassis->rl.max_pwm * speed_scale));
    Motor_SetSpeed(&chassis->rr, (int32_t)(rightRear * chassis->rr.max_pwm * speed_scale));
}

// Global encoder variables from main.c
/* encoder counters are declared in encoders.h and defined in encoders.c */

float Encoder_CalculateRPM(int32_t ticks_diff, float ppr, float gear_ratio, float dt_sec)
{
    if (dt_sec <= 0.0001f || ppr <= 0.0f || gear_ratio <= 0.0f) return 0.0f;
    
    // Total ticks per output wheel revolution in 4x quadrature mode
    float ticks_per_wheel_rev = ppr * 4.0f * gear_ratio;
    
    // RPM = (revolutions / second) * 60 seconds
    float rpm = ((float)ticks_diff / ticks_per_wheel_rev) * (60.0f / dt_sec);
    return rpm;
}

float Encoder_RPMToVelocity(float rpm, float wheel_diameter_m)
{
    // Circumference = pi * diameter
    float circumference = 3.1415926535f * wheel_diameter_m;
    
    // Velocity (m/s) = (RPM / 60) * circumference
    return (rpm / 60.0f) * circumference;
}

void Chassis_GetAndResetTicks(Mecanum_Chassis_t *chassis, int32_t *ticks_fl, int32_t *ticks_fr, int32_t *ticks_rl, int32_t *ticks_rr)
{
    if (chassis == NULL || ticks_fl == NULL || ticks_fr == NULL || ticks_rl == NULL || ticks_rr == NULL) return;

    // Read current values (already correctly oriented in the polling loop)
    *ticks_fl = enc4_count;
    *ticks_fr = enc1_count;
    *ticks_rl = enc3_count;
    *ticks_rr = enc2_count;
    
    // Reset global counters to 0
    enc1_count = 0;
    enc2_count = 0;
    enc3_count = 0;
    enc4_count = 0;
}
