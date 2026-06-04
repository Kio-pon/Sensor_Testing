#ifndef IMU_H
#define IMU_H

#include "stm32f3xx_hal.h"

#define I3G4250D_WHO_AM_I      0x0F
#define I3G4250D_CTRL_REG1     0x20
#define I3G4250D_CTRL_REG4     0x23
#define I3G4250D_OUT_X_L       0x28
#define I3G4250D_OUT_X_H       0x29
#define I3G4250D_OUT_Y_L       0x2A
#define I3G4250D_OUT_Y_H       0x2B
#define I3G4250D_OUT_Z_L       0x2C
#define I3G4250D_OUT_Z_H       0x2D

extern float gyro_yaw_deg;
extern float gyro_pitch_deg;
extern float gyro_roll_deg;

void IMU_Init(SPI_HandleTypeDef *hspi);
void IMU_ReadGyro(SPI_HandleTypeDef *hspi);

#endif /* IMU_H */
