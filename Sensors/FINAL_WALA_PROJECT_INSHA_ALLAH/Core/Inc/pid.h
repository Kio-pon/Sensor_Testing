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
    float kv;               /**< Feedforward Velocity Gain */
    
    float previous_error;   /**< Error in previous time step */
    float integral;         /**< Integrated error over time */
    
    float previous_derivative; /**< Filtered derivative from previous step */
    float filter_alpha;        /**< EMA filter alpha (0.0-1.0). 1.0 = no filter */
    
    float max_integral;     /**< Limit for integral windup protection */
    float max_output;       /**< Maximum output clamp */
} PID_t;

/**
 * @brief Initializes the PID controller with coefficients
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float kv, float max_integral, float max_output);

/**
 * @brief Updates the PID controller and calculates the correction output
 * @param pid Pointer to PID instance
 * @param error Current error value
 * @param dt Time delta since last update in seconds
 * @param target_velocity For Feedforward (Kv), set to 0.0f if unused
 * @return float PID correction output
 */
float PID_Update(PID_t *pid, float error, float dt, float target_velocity);

/**
 * @brief Sets the Exponential Moving Average filter alpha for the D-term
 * @param pid Pointer to PID instance
 * @param alpha 0.0 (heavy filtering) to 1.0 (no filtering)
 */
void PID_SetFilter(PID_t *pid, float alpha);

/**
 * @brief Resets the integral and previous error terms
 */
void PID_Reset(PID_t *pid);

#endif /* PID_H */
