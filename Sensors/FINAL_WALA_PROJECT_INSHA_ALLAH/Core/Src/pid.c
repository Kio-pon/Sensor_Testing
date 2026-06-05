#include "pid.h"
#include <stddef.h>   /* NULL */


void PID_Init(PID_t *pid, float kp, float ki, float kd, float kv, float max_integral, float max_output)
{
    if (pid == NULL) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kv = kv;
    
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->previous_derivative = 0.0f;
    pid->filter_alpha = 1.0f; // Default: no filter
    
    pid->max_integral = max_integral;
    pid->max_output = max_output;
}

void PID_SetFilter(PID_t *pid, float alpha)
{
    if (pid == NULL) return;
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    pid->filter_alpha = alpha;
}

float PID_Update(PID_t *pid, float error, float dt, float target_velocity)
{
    if (pid == NULL) return 0.0f;
    if (dt <= 0.0f) dt = 0.01f; // Safeguard against divide-by-zero or static delta
    
    // Proportional term
    float p_term = pid->kp * error;
    
    // Integral term with anti-windup clamping
    pid->integral += error * dt;
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }
    float i_term = pid->ki * pid->integral;
    
    // Derivative term with Low-Pass EMA Filter
    float raw_derivative = (error - pid->previous_error) / dt;
    float filtered_derivative = (pid->filter_alpha * raw_derivative) + ((1.0f - pid->filter_alpha) * pid->previous_derivative);
    float d_term = pid->kd * filtered_derivative;
    
    // Save state for next loop
    pid->previous_error = error;
    pid->previous_derivative = filtered_derivative;
    
    // Feedforward term
    float ff_term = pid->kv * target_velocity;
    
    // Calculate total correction and clamp output
    float output = p_term + i_term + d_term + ff_term;
    if (output > pid->max_output) {
        output = pid->max_output;
    } else if (output < -pid->max_output) {
        output = -pid->max_output;
    }
    
    return output;
}

void PID_Reset(PID_t *pid)
{
    if (pid == NULL) return;
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    pid->previous_derivative = 0.0f;
}
