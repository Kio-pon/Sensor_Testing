#include "robot_control.h"
#include <stdio.h>

/* Configurable robot speed parameters (out of motor max_pwm) */
#define SPEED_BASE_NORMAL 1200      // PWM speed for normal line following (out of e.g. 2000)
#define SPEED_BASE_RAMP   1800      // Increased speed/torque for the 22-degree incline climb
#define SPEED_BASE_SLOW   800       // Slow approach speed for alignment

/* Configurable shooter servo angles */
#define SHOOTER_RETRACT_DEG 0
#define SHOOTER_EXTEND_DEG  90

/* Elevator travel durations (ms) between consecutive slot heights */
#define ELEVATOR_SLOT_UP_MS 450
#define ELEVATOR_SLOT_DN_MS 400

/* Target turn counts (how many intersection ticks on side sensors before turning/stopping) */
#define TURNS_TO_S2 2
#define TURNS_TO_S3 3

void Robot_Init(RobotController_t *robot, 
                ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2,
                ADC_HandleTypeDef *hadc3, ADC_HandleTypeDef *hadc4,
                I2C_HandleTypeDef *hi2c_color,
                TIM_HandleTypeDef *htim_motors, TIM_HandleTypeDef *htim_servos)
{
    if (robot == NULL) return;
    
    robot->state = STATE_INIT;
    robot->arena_side = ARENA_LEFT; // Default, can be selected via GPIO jumper or UART
    robot->pallets_left = 6;
    robot->turn_count = 0;
    robot->state_timer = 0;
    
    /* ────────────────────────────────────────────────────────────────
     * Actuator & Driver Mappings (Assigned based on CubeMX configuration)
     * ──────────────────────────────────────────────────────────────── */
    
    // Front-Left Motor (Motor 1)
    robot->chassis.fl.IN1_Port = GPIOD;
    robot->chassis.fl.IN1_Pin  = GPIO_PIN_2;
    robot->chassis.fl.IN2_Port = GPIOD;
    robot->chassis.fl.IN2_Pin  = GPIO_PIN_1;
    robot->chassis.fl.htim     = htim_motors;
    robot->chassis.fl.channel  = TIM_CHANNEL_1;
    robot->chassis.fl.max_pwm  = 2000;
    
    // Front-Right Motor (Motor 2)
    robot->chassis.fr.IN1_Port = GPIOD;
    robot->chassis.fr.IN1_Pin  = GPIO_PIN_0;
    robot->chassis.fr.IN2_Port = GPIOC;
    robot->chassis.fr.IN2_Pin  = GPIO_PIN_12;
    robot->chassis.fr.htim     = htim_motors;
    robot->chassis.fr.channel  = TIM_CHANNEL_2;
    robot->chassis.fr.max_pwm  = 2000;
    
    // Rear-Left Motor (Motor 3)
    robot->chassis.rl.IN1_Port = GPIOC;
    robot->chassis.rl.IN1_Pin  = GPIO_PIN_10;
    robot->chassis.rl.IN2_Port = GPIOC;
    robot->chassis.rl.IN2_Pin  = GPIO_PIN_11;
    robot->chassis.rl.htim     = htim_motors;
    robot->chassis.rl.channel  = TIM_CHANNEL_3;
    robot->chassis.rl.max_pwm  = 2000;
    
    // Rear-Right Motor (Motor 4)
    robot->chassis.rr.IN1_Port = GPIOA;
    robot->chassis.rr.IN1_Pin  = GPIO_PIN_14;
    robot->chassis.rr.IN2_Port = GPIOA;
    robot->chassis.rr.IN2_Pin  = GPIO_PIN_15;
    robot->chassis.rr.htim     = htim_motors;
    robot->chassis.rr.channel  = TIM_CHANNEL_4;
    robot->chassis.rr.max_pwm  = 2000;
    
    // Motor driver Enable/Standby Pin
    robot->chassis.STBY_Port   = GPIOD;
    robot->chassis.STBY_Pin    = GPIO_PIN_5;
    
    Chassis_Init(&robot->chassis);
    
    // Elevator continuous rotation servo (Servo 1 -> TIM3 Channel 1)
    Elevator_Init(&robot->elevator, htim_servos, TIM_CHANNEL_1);
    robot->elevator.slot_up_duration = ELEVATOR_SLOT_UP_MS;
    robot->elevator.slot_dn_duration = ELEVATOR_SLOT_DN_MS;
    
    // Shooter standard servo (Servo 2 -> TIM3 Channel 2)
    robot->shooter.htim = htim_servos;
    robot->shooter.channel = TIM_CHANNEL_2;
    Servo_Init(&robot->shooter, SHOOTER_RETRACT_DEG);
    
    /* ────────────────────────────────────────────────────────────────
     * Sensor Configuration Mappings
     * ──────────────────────────────────────────────────────────────── */
    
    // Front QTR line array (8 channels, all on ADC1)
    ADC_HandleTypeDef *qtr_front_adcs[8] = {
        hadc1, hadc1, hadc1, hadc1, hadc1, hadc1, hadc1, hadc1
    };
    uint32_t qtr_front_ch[8] = {
        ADC_CHANNEL_2,  // PA1
        ADC_CHANNEL_3,  // PA2
        ADC_CHANNEL_4,  // PA3
        ADC_CHANNEL_6,  // PC0
        ADC_CHANNEL_7,  // PC1
        ADC_CHANNEL_8,  // PC2
        ADC_CHANNEL_9,  // PC3
        ADC_CHANNEL_5   // PF4
    };
    QTR_Init(&robot->qtr_front, qtr_front_adcs, qtr_front_ch, 8, QTR_DEFAULT_THRESHOLD);
    
    // Left QTR turn counter (6 channels, all on ADC3)
    ADC_HandleTypeDef *qtr_left_adcs[6] = {
        hadc3, hadc3, hadc3, hadc3, hadc3, hadc3
    };
    uint32_t qtr_left_ch[6] = {
        ADC_CHANNEL_1,  // PB1
        ADC_CHANNEL_2,  // PE9
        ADC_CHANNEL_3,  // PE13
        ADC_CHANNEL_5,  // PB13
        ADC_CHANNEL_6,  // PE8
        ADC_CHANNEL_7   // PD10
    };
    QTR_Init(&robot->qtr_left, qtr_left_adcs, qtr_left_ch, 6, QTR_DEFAULT_THRESHOLD);
    
    // Right QTR turn counter (6 channels, D1-D5 on ADC2, D6 on ADC3)
    ADC_HandleTypeDef *qtr_right_adcs[6] = {
        hadc2, hadc2, hadc2, hadc2, hadc2, hadc3
    };
    uint32_t qtr_right_ch[6] = {
        ADC_CHANNEL_1,  // PA4
        ADC_CHANNEL_2,  // PA5
        ADC_CHANNEL_3,  // PA6
        ADC_CHANNEL_4,  // PA7
        ADC_CHANNEL_5,  // PC4
        ADC_CHANNEL_8   // PD11
    };
    QTR_Init(&robot->qtr_right, qtr_right_adcs, qtr_right_ch, 6, QTR_DEFAULT_THRESHOLD);
    
    // Sharp Analog proximity sensors (all on ADC4)
    SharpSensor_Init(&robot->sharp_front, hadc4, ADC_CHANNEL_1); // PB12
    SharpSensor_Init(&robot->sharp_left,  hadc4, ADC_CHANNEL_2); // PE14
    SharpSensor_Init(&robot->sharp_right, hadc4, ADC_CHANNEL_3); // PE15
    
    // TCS34725 Color Sensor I2C Handle
    robot->hi2c_color = hi2c_color;
    TCS34725_Init(robot->hi2c_color);
    
    /* ────────────────────────────────────────────────────────────────
     * PID Line Tracking Initialization
     * ──────────────────────────────────────────────────────────────── */
    // Tuning values: Kp, Ki, Kd, max_integral, max_output
    // Target position is 3500 (centered). Steering output scales to motor differential
    PID_Init(&robot->line_pid, 0.45f, 0.02f, 0.15f, 500.0f, 600.0f);
}

void Robot_Update(RobotController_t *robot, float dt)
{
    if (robot == NULL) return;
    
    uint16_t qtr_front_raw[8];
    QTR_ReadRaw(&robot->qtr_front, qtr_front_raw);
    
    float position = QTR_GetLinePosition(&robot->qtr_front, qtr_front_raw);
    float target = 3500.0f; // Center of front 8-sensor array
    float error = target - position;
    float steer = PID_Update(&robot->line_pid, error, dt);
    
    float front_dist  = SharpSensor_ReadDistance(&robot->sharp_front);
    float left_dist   = SharpSensor_ReadDistance(&robot->sharp_left);
    float right_dist  = SharpSensor_ReadDistance(&robot->sharp_right);
    
    uint8_t left_turns_digital  = QTR_ReadDigital(&robot->qtr_left);
    uint8_t right_turns_digital = QTR_ReadDigital(&robot->qtr_right);
    
    robot->state_timer += (uint32_t)(dt * 1000.0f); // Accumulate time in state
    
    switch (robot->state) {
        
        case STATE_INIT:
            // Standby state, waiting for start trigger
            Chassis_Drive(&robot->chassis, 0, 0, 0);
            
            // Simple validation: if a line is detected on front array, whisle start!
            if (position != target || left_turns_digital != 0 || right_turns_digital != 0) {
                PID_Reset(&robot->line_pid);
                robot->state = STATE_MOVE_TO_S1;
                robot->state_timer = 0;
            }
            break;
            
        case STATE_MOVE_TO_S1:
            // Follow center line towards S1
            Chassis_Drive(&robot->chassis, SPEED_BASE_NORMAL, 0, (int32_t)steer);
            
            // Stop when we detect S1 support box in front
            if (front_dist < 13.0f) {
                Chassis_BrakeAll(&robot->chassis);
                HAL_Delay(300);
                robot->state = STATE_POTTING_S1;
                robot->state_timer = 0;
            }
            break;
            
        case STATE_POTTING_S1:
            // 1. Dynamic Open Face Search sequence:
            // S1 supporting box face is scanned. If closed, we strafe sideways or orbit
            // until color sensor detects blue mark or Sharp sensor reads slot opening depth.
            Chassis_Drive(&robot->chassis, 0, 0, 0);
            
            TCS34725_RawData color_data;
            TCS34725_ReadRaw(robot->hi2c_color, &color_data);
            DetectedColor face_color = TCS34725_ClassifyColor(&color_data);
            
            // If closed side, strafe right to check the next face
            uint8_t search_attempts = 0;
            while ((face_color != COLOR_BLUE && front_dist < 20.0f) && search_attempts < 4) {
                // Strafe right for 600ms to center on next face
                Chassis_Drive(&robot->chassis, 0, SPEED_BASE_SLOW, 0);
                HAL_Delay(600);
                Chassis_BrakeAll(&robot->chassis);
                HAL_Delay(300);
                
                // Read face again
                TCS34725_ReadRaw(robot->hi2c_color, &color_data);
                face_color = TCS34725_ClassifyColor(&color_data);
                front_dist = SharpSensor_ReadDistance(&robot->sharp_front);
                search_attempts++;
            }
            
            // 2. Perform potting sequence for S1 (2 pallets)
            // We scan vertical slots 0 and 1
            for (uint8_t slot = 0; slot < 2; slot++) {
                Elevator_MoveToSlot(&robot->elevator, slot);
                HAL_Delay(400);
                
                TCS34725_ReadRaw(robot->hi2c_color, &color_data);
                DetectedColor slot_color = TCS34725_ClassifyColor(&color_data);
                
                // S1 accepts pallets in any slot, but blue gives bonus points
                if (slot_color == COLOR_BLUE || slot_color == COLOR_UNKNOWN) {
                    Shooter_Fire(&robot->shooter, SHOOTER_RETRACT_DEG, SHOOTER_EXTEND_DEG);
                    robot->pallets_left--;
                }
            }
            
            // Lower elevator and proceed
            Elevator_MoveToSlot(&robot->elevator, 0);
            robot->state = STATE_MOVE_TO_S2;
            robot->state_timer = 0;
            robot->turn_count = 0;
            break;
            
        case STATE_MOVE_TO_S2:
            // Navigate towards the left wall (S2). 
            // Strafe sideways and line follow using simple bypass strategy.
            Chassis_Drive(&robot->chassis, SPEED_BASE_NORMAL, 0, (int32_t)steer);
            
            // Count junction crossings using left side sensors
            // Standard crossing tick: multiple side sensors black
            if (left_turns_digital & 0x1C) { // check center 3 side sensors
                robot->turn_count++;
                HAL_Delay(300); // debounce junction to avoid double counting
            }
            
            // Stop when we reach S2 wall
            if (left_dist < 12.0f || robot->turn_count >= TURNS_TO_S2) {
                Chassis_BrakeAll(&robot->chassis);
                HAL_Delay(300);
                robot->state = STATE_POTTING_S2;
                robot->state_timer = 0;
            }
            break;
            
        case STATE_POTTING_S2:
            Chassis_Drive(&robot->chassis, 0, 0, 0);
            
            // S2 is fixed to left wall. Scan 4 horizontal slots.
            // We use Mecanum strafe to step from slot 0 to slot 3.
            uint8_t s2_potted = 0;
            for (uint8_t slot = 0; slot < 4 && s2_potted < 2; slot++) {
                // Strafe sideways to align with next slot (e.g. 500ms strafe step)
                if (slot > 0) {
                    Chassis_Drive(&robot->chassis, 0, SPEED_BASE_SLOW, 0); // Strafe right
                    HAL_Delay(500);
                    Chassis_BrakeAll(&robot->chassis);
                    HAL_Delay(200);
                }
                
                TCS34725_ReadRaw(robot->hi2c_color, &color_data);
                DetectedColor slot_color = TCS34725_ClassifyColor(&color_data);
                
                if (slot_color == COLOR_BLUE) {
                    // Match elevator height to the horizontal slot line
                    Elevator_MoveToSlot(&robot->elevator, 1); // target slot height
                    HAL_Delay(400);
                    
                    Shooter_Fire(&robot->shooter, SHOOTER_RETRACT_DEG, SHOOTER_EXTEND_DEG);
                    robot->pallets_left--;
                    s2_potted++;
                }
            }
            
            Elevator_MoveToSlot(&robot->elevator, 0);
            robot->state = STATE_MOVE_TO_S3;
            robot->state_timer = 0;
            robot->turn_count = 0;
            break;
            
        case STATE_MOVE_TO_S3:
            // S3 sits at the top of a 22-degree ramp on the right side.
            // Simple bypass strategy: strafe/turn onto the straight right bypass lane
            
            // Check if climbing the incline: Front distance is close or pitch is steep
            if (front_dist < 20.0f && right_dist < 20.0f) {
                // Climb! Increase PWM base power for high torque climb
                Chassis_Drive(&robot->chassis, SPEED_BASE_RAMP, 0, (int32_t)steer);
            } else {
                Chassis_Drive(&robot->chassis, SPEED_BASE_NORMAL, 0, (int32_t)steer);
            }
            
            // Arrived at S3 at top of ramp
            if (front_dist < 12.0f) {
                Chassis_BrakeAll(&robot->chassis);
                HAL_Delay(300);
                robot->state = STATE_POTTING_S3;
                robot->state_timer = 0;
            }
            break;
            
        case STATE_POTTING_S3:
            Chassis_Drive(&robot->chassis, 0, 0, 0);
            
            // Scan S3's 4 horizontal slots at the top of the ramp
            uint8_t s3_potted = 0;
            for (uint8_t slot = 0; slot < 4 && s3_potted < 2; slot++) {
                if (slot > 0) {
                    Chassis_Drive(&robot->chassis, 0, -SPEED_BASE_SLOW, 0); // Strafe left
                    HAL_Delay(500);
                    Chassis_BrakeAll(&robot->chassis);
                    HAL_Delay(200);
                }
                
                TCS34725_ReadRaw(robot->hi2c_color, &color_data);
                DetectedColor slot_color = TCS34725_ClassifyColor(&color_data);
                
                if (slot_color == COLOR_BLUE) {
                    Elevator_MoveToSlot(&robot->elevator, 2); // S3 elevated height level
                    HAL_Delay(400);
                    
                    Shooter_Fire(&robot->shooter, SHOOTER_RETRACT_DEG, SHOOTER_EXTEND_DEG);
                    robot->pallets_left--;
                    s3_potted++;
                }
            }
            
            Elevator_MoveToSlot(&robot->elevator, 0);
            robot->state = STATE_PARKING;
            robot->state_timer = 0;
            break;
            
        case STATE_PARKING:
            // S3 has been cleared. Move backward completely inside the parking zone boundary.
            // Move backward at base speed for 1.2s to fully clear the line
            Chassis_Drive(&robot->chassis, -SPEED_BASE_NORMAL, 0, 0);
            HAL_Delay(1200);
            
            Chassis_BrakeAll(&robot->chassis);
            robot->state = STATE_FINISHED;
            robot->state_timer = 0;
            break;
            
        case STATE_FINISHED:
            // Completely stop all motors and servos. End of warehouse run.
            Chassis_Drive(&robot->chassis, 0, 0, 0);
            Chassis_SetStandby(&robot->chassis, 0); // De-energize drivers
            Elevator_Stop(&robot->elevator);
            break;
    }
}

void Robot_EmergencyStop(RobotController_t *robot)
{
    if (robot == NULL) return;
    
    Chassis_BrakeAll(&robot->chassis);
    Chassis_SetStandby(&robot->chassis, 0);
    Elevator_Stop(&robot->elevator);
    robot->state = STATE_FINISHED;
}
