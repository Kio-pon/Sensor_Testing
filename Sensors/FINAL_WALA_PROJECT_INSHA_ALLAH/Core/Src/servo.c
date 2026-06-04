#include "servo.h"

#define CR_SERVO_STOP_TICK 1500   // 1.5ms pulse (Stop)
#define CR_SERVO_UP_TICK   1000   // 1.0ms pulse (Max speed CW / UP)
#define CR_SERVO_DOWN_TICK 2000   // 2.0ms pulse (Max speed CCW / DOWN)

void Servo_Init(Servo_t *servo, uint8_t default_angle)
{
    if (servo == NULL || servo->htim == NULL) return;
    
    HAL_TIM_PWM_Start(servo->htim, servo->channel);
    Servo_SetAngle(servo, default_angle);
}

void Servo_SetAngle(Servo_t *servo, uint8_t angle)
{
    if (servo == NULL || servo->htim == NULL) return;
    
    if (angle > 180) angle = 180;
    
    // Standard servo translation formula:
    // 0 degrees   -> 1000us pulse width
    // 180 degrees -> 2000us pulse width
    uint32_t pulse = 1000 + ((uint32_t)angle * 1000 / 180);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse);
}

void Elevator_Init(Elevator_t *elevator, TIM_HandleTypeDef *htim, uint32_t channel)
{
    if (elevator == NULL || htim == NULL) return;
    
    elevator->servo.htim = htim;
    elevator->servo.channel = channel;
    
    // Default calibration values
    elevator->current_slot = 0; // Starts at bottom slot
    elevator->slot_up_duration = 450; // 450ms per slot upward travel
    elevator->slot_dn_duration = 400; // 400ms per slot downward travel
    
    HAL_TIM_PWM_Start(elevator->servo.htim, elevator->servo.channel);
    Elevator_Stop(elevator);
}

void Elevator_MoveToSlot(Elevator_t *elevator, uint8_t target_slot)
{
    if (elevator == NULL || elevator->servo.htim == NULL) return;
    if (target_slot > 3) target_slot = 3; // 4 slots (0, 1, 2, 3)
    
    if (target_slot > elevator->current_slot) {
        // Move UP
        uint8_t diff = target_slot - elevator->current_slot;
        uint32_t duration = (uint32_t)diff * elevator->slot_up_duration;
        
        // Spin continuous rotation servo UP
        __HAL_TIM_SET_COMPARE(elevator->servo.htim, elevator->servo.channel, CR_SERVO_UP_TICK);
        HAL_Delay(duration);
        
        Elevator_Stop(elevator);
        elevator->current_slot = target_slot;
    }
    else if (target_slot < elevator->current_slot) {
        // Move DOWN
        uint8_t diff = elevator->current_slot - target_slot;
        uint32_t duration = (uint32_t)diff * elevator->slot_dn_duration;
        
        // Spin continuous rotation servo DOWN
        __HAL_TIM_SET_COMPARE(elevator->servo.htim, elevator->servo.channel, CR_SERVO_DOWN_TICK);
        HAL_Delay(duration);
        
        Elevator_Stop(elevator);
        elevator->current_slot = target_slot;
    }
}

void Elevator_Stop(Elevator_t *elevator)
{
    if (elevator == NULL || elevator->servo.htim == NULL) return;
    __HAL_TIM_SET_COMPARE(elevator->servo.htim, elevator->servo.channel, CR_SERVO_STOP_TICK);
}

void Shooter_Fire(Servo_t *shooter, uint8_t retract_deg, uint8_t extend_deg)
{
    if (shooter == NULL || shooter->htim == NULL) return;
    
    // 1. Move to extended angle to push the pallet
    Servo_SetAngle(shooter, extend_deg);
    HAL_Delay(500); // Wait for the linkage to fully extend and seat the pallet
    
    // 2. Retract shooter back to holding position
    Servo_SetAngle(shooter, retract_deg);
    HAL_Delay(400); // Wait for shooter pin to fully clear the slot
}
