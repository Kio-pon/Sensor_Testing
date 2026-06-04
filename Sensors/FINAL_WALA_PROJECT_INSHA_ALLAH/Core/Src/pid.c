#include "pid.h"
#include <stddef.h>   /* NULL */


void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_integral, float max_output)
{
    if (pid == NULL) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    
    pid->previous_error = 0.0f;
    pid->integral = 0.0f;
    
    pid->max_integral = max_integral;
    pid->max_output = max_output;
}

float PID_Update(PID_t *pid, float error, float dt)
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
    
    // Derivative term
    float derivative = (error - pid->previous_error) / dt;
    float d_term = pid->kd * derivative;
    
    // Save current error for next loop
    pid->previous_error = error;
    
    // Calculate total correction and clamp output
    float output = p_term + i_term + d_term;
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
}
