#ifndef SERVO_H
#define SERVO_H

#include "main.h"

/**
 * @brief Standard Servo structure
 */
typedef struct {
    TIM_HandleTypeDef *htim;    /**< PWM Timer handle pointer */
    uint32_t channel;           /**< PWM Timer channel */
} Servo_t;

/**
 * @brief Continuous Rotation Elevator Servo structure
 */
typedef struct {
    Servo_t servo;              /**< Underlying PWM servo */
    uint8_t current_slot;       /**< Tracked elevator slot height (0 to 3) */
    uint32_t slot_up_duration;  /**< Travel time in ms between slots moving UP */
    uint32_t slot_dn_duration;  /**< Travel time in ms between slots moving DOWN */
} Elevator_t;

/**
 * @brief Initializes a standard servo to a default angle
 */
void Servo_Init(Servo_t *servo, uint8_t default_angle);

/**
 * @brief Sets a standard servo's angle (0 to 180 degrees)
 */
void Servo_SetAngle(Servo_t *servo, uint8_t angle);

/**
 * @brief Initializes the continuous rotation elevator servo
 */
void Elevator_Init(Elevator_t *elevator, TIM_HandleTypeDef *htim, uint32_t channel);

/**
 * @brief Moves the time-based elevator to a specific slot level (0 to 3)
 */
void Elevator_MoveToSlot(Elevator_t *elevator, uint8_t target_slot);

/**
 * @brief Stops the continuous rotation elevator instantly
 */
void Elevator_Stop(Elevator_t *elevator);

/**
 * @brief Fire/deploy the shooting servo (pushes a pallet and retracts)
 * @param shooter The standard servo instance
 * @param retract_deg Angle for retracted/idle state
 * @param extend_deg Angle for extended/firing state
 */
void Shooter_Fire(Servo_t *shooter, uint8_t retract_deg, uint8_t extend_deg);

#endif /* SERVO_H */
