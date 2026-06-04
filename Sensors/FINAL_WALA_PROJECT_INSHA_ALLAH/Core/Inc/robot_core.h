 #ifndef ROBOT_CORE_H
#define ROBOT_CORE_H

#include "main.h"

#include "robot_config.h"

void Robot_Init(void);
void Robot_RunLoop(void);

void Turn90_LineSnap(int8_t dir);
void Turn_ByTicks(int8_t dir, int32_t ticks);
int32_t Strafe_CM_To_Ticks(float cm);

#endif /* ROBOT_CORE_H */
