 #ifndef ROBOT_CORE_H
#define ROBOT_CORE_H

#include "main.h"
#include <stdbool.h>
#include "pid.h"
#include "robot_config.h"

extern float target_heading;
extern int32_t robot_vx;
extern int32_t robot_vy;
extern PID_t gyro_pid;
extern int32_t nav_omega_override;
extern bool use_nav_omega;

/* Absolute Odometry (Grid Position) */
extern float global_x; // mm
extern float global_y; // mm

void Robot_Init(void);
void Robot_RunLoop(void);

void Turn90_LineSnap(int8_t dir);
void Turn_ByTicks(int8_t dir, int32_t ticks);
int32_t Strafe_CM_To_Ticks(float cm);

#endif /* ROBOT_CORE_H */
