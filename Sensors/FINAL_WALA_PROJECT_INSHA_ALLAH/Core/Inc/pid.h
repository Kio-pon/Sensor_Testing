#ifndef PID_H
#define PID_H

#include <stdint.h>

/**
 * @brief PID Controller structure
 */
typedef struct {
    float kp;               /**< Proportional Gain */
    float ki;               /**< Integral Gain */
    float kd;               /**< Derivative Gain */
    
    float previous_error;   /**< Error in previous time step */
    float integral;         /**< Integrated error over time */
    
    float max_integral;     /**< Limit for integral windup protection */
    float max_output;       /**< Maximum output clamp */
} PID_t;

/**
 * @brief Initializes the PID controller with coefficients
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_integral, float max_output);

/**
 * @brief Updates the PID controller and calculates the correction output
 * @param pid Pointer to PID instance
 * @param error Current error value
 * @param dt Time delta since last update in seconds
 * @return float PID correction output
 */
float PID_Update(PID_t *pid, float error, float dt);

/**
 * @brief Resets the integral and previous error terms
 */
void PID_Reset(PID_t *pid);

#endif /* PID_H */
