#include "MecanumDrive.h"
#include "Config.h"

void initMotors() {
  uint8_t outputPins[] = {
    FL_ENA, FL_IN1, FL_IN2,
    FR_ENA, FR_IN1, FR_IN2,
    RL_ENB, RL_IN3, RL_IN4,
    RR_ENB, RR_IN3, RR_IN4
  };

  for (int i = 0; i < 12; i++) {
    pinMode(outputPins[i], OUTPUT);
    digitalWrite(outputPins[i], LOW);
  }
}

// FIXED KINEMATICS: Wheels/Motors are mounted 90 degrees off.

void driveForward(int leftPwm, int rightPwm) {
  // Originally strafeRight. To implement left/right turning while driving forward (which is now physically strafeRight logic):
  // Forward logic means FL and RL turn against each other, FR and RR turn against each other.
  // We apply the differential leftPwm to FL/RL and rightPwm to FR/RR.
  
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);  analogWrite(FL_ENA, leftPwm);
  digitalWrite(RL_IN3, LOW);  digitalWrite(RL_IN4, HIGH); analogWrite(RL_ENB, leftPwm);
  
  digitalWrite(FR_IN1, LOW);  digitalWrite(FR_IN2, HIGH); analogWrite(FR_ENA, rightPwm);
  digitalWrite(RR_IN3, HIGH); digitalWrite(RR_IN4, LOW);  analogWrite(RR_ENB, rightPwm);
}

void driveBackward(int leftPwm, int rightPwm) {
  digitalWrite(FL_IN1, LOW);  digitalWrite(FL_IN2, HIGH); analogWrite(FL_ENA, leftPwm);
  digitalWrite(RL_IN3, HIGH); digitalWrite(RL_IN4, LOW);  analogWrite(RL_ENB, leftPwm);
  
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);  analogWrite(FR_ENA, rightPwm);
  digitalWrite(RR_IN3, LOW);  digitalWrite(RR_IN4, HIGH); analogWrite(RR_ENB, rightPwm);
}

void strafeRight(int pwm) {
  // Originally driveBackward (all wheels backward)
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH); analogWrite(FL_ENA, pwm);
  digitalWrite(RL_IN3, LOW); digitalWrite(RL_IN4, HIGH); analogWrite(RL_ENB, pwm);
  
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH); analogWrite(FR_ENA, pwm);
  digitalWrite(RR_IN3, LOW); digitalWrite(RR_IN4, HIGH); analogWrite(RR_ENB, pwm);
}

void strafeLeft(int pwm) {
  // Originally driveForward (all wheels forward)
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW); analogWrite(FL_ENA, pwm);
  digitalWrite(RL_IN3, HIGH); digitalWrite(RL_IN4, LOW); analogWrite(RL_ENB, pwm);
  
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW); analogWrite(FR_ENA, pwm);
  digitalWrite(RR_IN3, HIGH); digitalWrite(RR_IN4, LOW); analogWrite(RR_ENB, pwm);
}

void spinRight(int pwm) {
  // Due to 90 degree offset, Front moving Right and Rear moving Left causes Clockwise Yaw.
  // FL/FR = Right (Fwd). RL/RR = Left (Bwd).
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);  analogWrite(FL_ENA, pwm);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);  analogWrite(FR_ENA, pwm);
  digitalWrite(RL_IN3, LOW);  digitalWrite(RL_IN4, HIGH); analogWrite(RL_ENB, pwm);
  digitalWrite(RR_IN3, LOW);  digitalWrite(RR_IN4, HIGH); analogWrite(RR_ENB, pwm);
}

void spinLeft(int pwm) {
  // FL/FR = Left (Bwd). RL/RR = Right (Fwd).
  digitalWrite(FL_IN1, LOW);  digitalWrite(FL_IN2, HIGH); analogWrite(FL_ENA, pwm);
  digitalWrite(FR_IN1, LOW);  digitalWrite(FR_IN2, HIGH); analogWrite(FR_ENA, pwm);
  digitalWrite(RL_IN3, HIGH); digitalWrite(RL_IN4, LOW);  analogWrite(RL_ENB, pwm);
  digitalWrite(RR_IN3, HIGH); digitalWrite(RR_IN4, LOW);  analogWrite(RR_ENB, pwm);
}

void stopAll() {
  analogWrite(FL_ENA, 0); analogWrite(RL_ENB, 0);
  analogWrite(FR_ENA, 0); analogWrite(RR_ENB, 0);
  
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(RL_IN3, LOW); digitalWrite(RL_IN4, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  digitalWrite(RR_IN3, LOW); digitalWrite(RR_IN4, LOW);
}
