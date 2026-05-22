#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "main.h"

/**
 * @brief TB6612FNG Single Motor structure
 */
typedef struct {
    GPIO_TypeDef *IN1_Port;     /**< Direction Input 1 GPIO Port */
    uint16_t IN1_Pin;           /**< Direction Input 1 GPIO Pin */
    GPIO_TypeDef *IN2_Port;     /**< Direction Input 2 GPIO Port */
    uint16_t IN2_Pin;           /**< Direction Input 2 GPIO Pin */
    TIM_HandleTypeDef *htim;    /**< PWM Timer Handle */
    uint32_t channel;           /**< PWM Timer Channel */
    uint32_t max_pwm;           /**< Maximum Timer Compare value (Period) */
} TB6612_Motor_t;

/**
 * @brief 4-Motor Mecanum Chassis structure
 */
typedef struct {
    TB6612_Motor_t fl;          /**< Front-Left Motor */
    TB6612_Motor_t fr;          /**< Front-Right Motor */
    TB6612_Motor_t rl;          /**< Rear-Left Motor */
    TB6612_Motor_t rr;          /**< Rear-Right Motor */
    GPIO_TypeDef *STBY_Port;    /**< Standby Pin GPIO Port */
    uint16_t STBY_Pin;          /**< Standby Pin GPIO Pin */
} Mecanum_Chassis_t;

/**
 * @brief Initializes a single motor channel
 */
void Motor_Init(TB6612_Motor_t *motor);

/**
 * @brief Sets speed and direction of a single motor channel
 * @param speed Range: [-max_pwm, max_pwm]
 */
void Motor_SetSpeed(TB6612_Motor_t *motor, int32_t speed);

/**
 * @brief Activates short braking mode for a single motor (fast stop)
 */
void Motor_Brake(TB6612_Motor_t *motor);

/**
 * @brief Stops a motor immediately (coast/standby)
 */
void Motor_Stop(TB6612_Motor_t *motor);

/**
 * @brief Initializes the entire Mecanum chassis
 */
void Chassis_Init(Mecanum_Chassis_t *chassis);

/**
 * @brief Sets standby state of the TB6612FNG driver
 * @param state 1 to enable motors (STBY High), 0 to disable (STBY Low)
 */
void Chassis_SetStandby(Mecanum_Chassis_t *chassis, uint8_t state);

/**
 * @brief Command wheel speeds using Mecanum kinematics
 * @param vx Forward/reverse speed (X-axis)
 * @param vy Strafe/sideways speed (Y-axis)
 * @param omega Rotational yaw speed (rotation)
 */
void Chassis_Drive(Mecanum_Chassis_t *chassis, int32_t vx, int32_t vy, int32_t omega);

/**
 * @brief Brakes all four motors instantly to stop on target lines
 */
void Chassis_BrakeAll(Mecanum_Chassis_t *chassis);

/**
 * @brief Drives the robot forward, brakes, then backward, then brakes.
 * @param speed Range: [0, max_pwm]
 * @param turn Range: [-max_pwm, max_pwm]
 * @param duration_ms Time to drive in each direction
 */
void Chassis_DriveForwardBackward(Mecanum_Chassis_t *chassis, int32_t speed, int32_t turn, uint32_t duration_ms);

/**
 * @brief Strafes the robot right, brakes, then left, then brakes.
 * @param speed Range: [0, max_pwm]
 * @param turn Range: [-max_pwm, max_pwm]
 * @param duration_ms Time to strafe in each direction
 */
void Chassis_DriveStrafe(Mecanum_Chassis_t *chassis, int32_t speed, int32_t turn, uint32_t duration_ms);

/**
 * @brief Drives the robot along the four diagonals in sequence.
 * @param speed Range: [0, max_pwm]
 * @param turn Range: [-max_pwm, max_pwm]
 * @param duration_ms Time to drive along each diagonal
 */
void Chassis_DriveDiagonals(Mecanum_Chassis_t *chassis, int32_t speed, int32_t turn, uint32_t duration_ms);

/**
 * @brief Drives the robot at any arbitrary angle relative to the chassis using trigonometric kinematics.
 * @param speed Range: [0, max_pwm]
 * @param turn Range: [-max_pwm, max_pwm]
 * @param angle_deg Direction of movement in degrees (0 = Strafe Right, 90 = Forward, 180 = Strafe Left, 270 = Backward)
 * @param duration_ms Time to drive at this angle
 */
void Chassis_DriveArbitraryAngle(Mecanum_Chassis_t *chassis, int32_t speed, int32_t turn, float angle_deg, uint32_t duration_ms);

/**
 * @brief Drives the robot back-and-forth along the diameter of a circle at a specified angle.
 * @param speed Range: [0, max_pwm]
 * @param turn Range: [-max_pwm, max_pwm]
 * @param diameter_angle_deg The angle of the diameter line in degrees (e.g. 90 = Up-Down diameter, 0 = Left-Right diameter)
 * @param duration_ms Time to drive across the diameter in one direction
 */
void Chassis_DriveCircleDiameter(Mecanum_Chassis_t *chassis, int32_t speed, int32_t turn, float diameter_angle_deg, uint32_t duration_ms);

/**
 * @brief Pivot rotates the robot in place clockwise, brakes, then counter-clockwise, then brakes.
 * @param turn Pivot turn speed/PWM magnitude. Range: [0, max_pwm]
 * @param duration_ms Time to rotate in each direction
 */
void Chassis_RotateInPlace(Mecanum_Chassis_t *chassis, int32_t turn, uint32_t duration_ms);

/**
 * @brief Drives the robot forward and backward with a smooth PWM ramp-up and ramp-down.
 * @param target_speed Peak speed/PWM to ramp up to. Range: [0, max_pwm]
 * @param turn Turn bias. Range: [-max_pwm, max_pwm]
 * @param duration_ms Duration to run at the peak target_speed
 * @param ramp_time_ms Duration spent ramping up and ramping down
 */
void Chassis_DriveForwardBackwardRamped(Mecanum_Chassis_t *chassis, int32_t target_speed, int32_t turn, uint32_t duration_ms, uint32_t ramp_time_ms);

#endif /* MOTOR_DRIVER_H */
