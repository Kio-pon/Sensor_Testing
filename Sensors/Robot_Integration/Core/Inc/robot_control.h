#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include "main.h"
#include "motor_driver.h"
#include "qtr_8a.h"
#include "sharp_sensor.h"
#include "tcs34725.h"
#include "servo.h"
#include "pid.h"

/**
 * @brief Autonomous robot run states
 */
typedef enum {
    STATE_INIT = 0,
    STATE_MOVE_TO_S1,
    STATE_POTTING_S1,
    STATE_MOVE_TO_S2,
    STATE_POTTING_S2,
    STATE_MOVE_TO_S3,
    STATE_POTTING_S3,
    STATE_PARKING,
    STATE_FINISHED
} RobotState_t;

/**
 * @brief Arena configuration side
 */
typedef enum {
    ARENA_LEFT = 0,     /**< Blue flag, left side warehouse */
    ARENA_RIGHT         /**< Red flag, right side warehouse */
} ArenaSide_t;

/**
 * @brief Master Robot Control structure
 */
typedef struct {
    RobotState_t state;             /**< Current execution state */
    ArenaSide_t arena_side;         /**< Configured arena mirror side */
    
    Mecanum_Chassis_t chassis;      /**< Mecanum drive chassis */
    
    QTR_Array_t qtr_front;          /**< Front line tracker (8 ch) */
    QTR_Array_t qtr_left;           /**< Left count/turn tracker (6 ch) */
    QTR_Array_t qtr_right;          /**< Right count/turn tracker (6 ch) */
    
    SharpSensor_t sharp_front;      /**< Front IR distance */
    SharpSensor_t sharp_left;       /**< Left IR distance */
    SharpSensor_t sharp_right;      /**< Right IR distance */
    
    I2C_HandleTypeDef *hi2c_color;  /**< I2C handle for color sensor */
    
    Elevator_t elevator;            /**< Elevator continuous rotation servo */
    Servo_t shooter;                /**< Standard shooter servo */
    
    PID_t line_pid;                 /**< PID loop for front line tracking */
    
    uint8_t pallets_left;           /**< Total pallets left (initially 6) */
    uint8_t turn_count;             /**< Turn/junction counter */
    uint32_t state_timer;           /**< Elapsed state duration tracking */
} RobotController_t;

/**
 * @brief Initializes the entire autonomous controller and hardware mappings
 */
void Robot_Init(RobotController_t *robot, 
                ADC_HandleTypeDef *hadc1, ADC_HandleTypeDef *hadc2,
                ADC_HandleTypeDef *hadc3, ADC_HandleTypeDef *hadc4,
                I2C_HandleTypeDef *hi2c_color,
                TIM_HandleTypeDef *htim_motors, TIM_HandleTypeDef *htim_servos);


/**
 * @brief Execution step loop called by main loop. Operates the state machine.
 * @param dt Time delta since last update loop in seconds (typically ~0.01s for 100Hz)
 */
void Robot_Update(RobotController_t *robot, float dt);

/**
 * @brief Force-stops all motion and resets actuator parameters
 */
void Robot_EmergencyStop(RobotController_t *robot);

#endif /* ROBOT_CONTROL_H */
