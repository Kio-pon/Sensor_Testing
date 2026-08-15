#ifndef NAVIGATION_H
#define NAVIGATION_H

#include "stm32f3xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    NAV_IDLE = 0,
    NAV_MOVE_DIST,
    NAV_FOLLOW_LINE_DIST,
    NAV_STRAFE_LOCKED,
    NAV_CHECK_COLOR,
    NAV_GOTO_COORD,
    NAV_TURN,
    NAV_FOLLOW_LINE_KEEP_DIST,
    NAV_STOP
} NavState_t;

/* Initialize navigation PID controllers and state */
void Nav_Init(void);

/* Called every 5ms in Robot_RunLoop. Returns true if busy, false if idle */
bool Nav_Update(void);

/* The hardcoded sequence function to be called continuously in the loop */
void Nav_RunSequence(void);

/* Non-blocking commands to start an action */
void Nav_StartMove(float cm, int32_t vx, int32_t vy);
void Nav_StrafeAlongLineUntilJunction(int32_t speed_vx, float min_cm);
void Nav_StrafeAlongLineForDistance(float cm, int32_t speed_vx);
void Nav_StartColorCheck(void);
void Nav_GoToCoordinate(float target_x_mm, float target_y_mm, int32_t speed);
void Nav_FollowLineForDistance(float cm, int32_t speed);
void Nav_FollowLineUntilDistance(float target_cm, int32_t speed);
void Nav_StartTurn(float degrees, int32_t speed);
void Nav_Stop(void);

/* Check if the state machine is currently executing an action */
bool Nav_IsBusy(void);

#endif /* NAVIGATION_H */
