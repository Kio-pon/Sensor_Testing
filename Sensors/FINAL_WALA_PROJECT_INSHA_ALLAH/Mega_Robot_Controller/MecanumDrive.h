#ifndef MECANUM_DRIVE_H
#define MECANUM_DRIVE_H

#include <Arduino.h>

void initMotors();
void driveForward(int leftPWM, int rightPWM);
void driveBackward(int leftPWM, int rightPWM);
void stopAll();

// Advanced Omni-Directional Kinematics
void strafeRight(int pwm);
void strafeLeft(int pwm);
void spinRight(int pwm);
void spinLeft(int pwm);
void rotateClockwise(int pwm);
void rotateCounterClockwise(int pwm);

#endif
