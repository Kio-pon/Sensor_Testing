/*
 * NERC 2026 - Encoder & Mecanum Kinematics Calibration Test
 * Moves Forward 50cm, Backward 50cm, Right 50cm, Left 50cm.
 */

#include <Arduino.h>

// --- MOTOR PINS ---
#define FL_ENA  2
#define FL_IN1  22
#define FL_IN2  23

#define RL_ENB  3
#define RL_IN3  24
#define RL_IN4  25

#define FR_ENA  4
#define FR_IN1  26
#define FR_IN2  27

#define RR_ENB  5
#define RR_IN3  28
#define RR_IN4  29

// --- ENCODER PINS (PORTB) ---
#define FL_ENCODER_A 50
#define FL_ENCODER_B 51
#define FR_ENCODER_A 52
#define FR_ENCODER_B 53
#define RL_ENCODER_A 10
#define RL_ENCODER_B 11
#define RR_ENCODER_A 12
#define RR_ENCODER_B 13

// --- PHYSICAL CONSTANTS (Matched to STM32) ---
#define WHEEL_DIAMETER_MM    80.0f
#define ENCODER_PPR          11.0f
#define GEAR_RATIO           19.7f
#define TICKS_PER_WHEEL_REV  (ENCODER_PPR * 4.0f * GEAR_RATIO) // ~866.8

#define WHEEL_CIRC_CM        (3.14159265f * WHEEL_DIAMETER_MM / 10.0f) // ~25.13
const float TICKS_PER_CM = (TICKS_PER_WHEEL_REV / WHEEL_CIRC_CM); // ~34.5 ticks per cm

#define MOVE_SPEED 150

volatile long countFL = 0;
volatile long countFR = 0;
volatile long countRL = 0;
volatile long countRR = 0;
static uint8_t lastPortB = 0;

void driveForward(int pwm);
void driveBackward(int pwm);
void strafeRight(int pwm);
void strafeLeft(int pwm);
void stopAll();
void moveDistanceTarget(float cm, int direction);

void setup() {
  Serial.begin(115200);

  // Motor Init
  uint8_t dirPins[] = {FL_IN1, FL_IN2, RL_IN3, RL_IN4, FR_IN1, FR_IN2, RR_IN3, RR_IN4};
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(dirPins[i], OUTPUT);
    digitalWrite(dirPins[i], LOW);
  }
  pinMode(FL_ENA, OUTPUT); pinMode(RL_ENB, OUTPUT);
  pinMode(FR_ENA, OUTPUT); pinMode(RR_ENB, OUTPUT);

  // Encoder Init
  pinMode(FL_ENCODER_A, INPUT_PULLUP); pinMode(FL_ENCODER_B, INPUT_PULLUP);
  pinMode(FR_ENCODER_A, INPUT_PULLUP); pinMode(FR_ENCODER_B, INPUT_PULLUP);
  pinMode(RL_ENCODER_A, INPUT_PULLUP); pinMode(RL_ENCODER_B, INPUT_PULLUP);
  pinMode(RR_ENCODER_A, INPUT_PULLUP); pinMode(RR_ENCODER_B, INPUT_PULLUP);

  PCICR |= (1 << PCIE0); 
  PCMSK0 |= 0xFF;
  lastPortB = PINB;

  Serial.println("Starting in 3 seconds...");
  delay(3000); 

  // The Test Routine
  Serial.println("Moving Forward 50cm...");
  moveDistanceTarget(50.0, 1);  // 1 = Forward
  delay(1000);

  Serial.println("Moving Backward 50cm...");
  moveDistanceTarget(50.0, 2);  // 2 = Backward
  delay(1000);

  Serial.println("Strafing Right 50cm...");
  moveDistanceTarget(50.0, 3);  // 3 = Right
  delay(1000);

  Serial.println("Strafing Left 50cm...");
  moveDistanceTarget(50.0, 4);  // 4 = Left
  delay(1000);

  Serial.println("Test Complete.");
}

void loop() {
  // Do nothing, run once
}

// Direction: 1=Fwd, 2=Bwd, 3=Right, 4=Left
void moveDistanceTarget(float cm, int direction) {
  long targetTicks = (long)(cm * TICKS_PER_CM);
  
  // Reset Encoders
  noInterrupts();
  countFL = 0; countFR = 0; countRL = 0; countRR = 0;
  interrupts();

  while(true) {
    long avgTicks = (abs(countFL) + abs(countFR) + abs(countRL) + abs(countRR)) / 4;
    
    if (avgTicks >= targetTicks) {
      stopAll();
      break;
    }

    if (direction == 1) driveForward(MOVE_SPEED);
    else if (direction == 2) driveBackward(MOVE_SPEED);
    else if (direction == 3) strafeRight(MOVE_SPEED);
    else if (direction == 4) strafeLeft(MOVE_SPEED);
  }
}

// FIXED KINEMATICS: 
// Physical wheels were mounted/wired 90 degrees offset. 
// What used to be "strafe right" physically causes "drive forward".

void driveForward(int pwm) {
  // Originally strafeRight: FL Fwd, RL Bwd, FR Bwd, RR Fwd
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);  analogWrite(FL_ENA, pwm);
  digitalWrite(RL_IN3, LOW);  digitalWrite(RL_IN4, HIGH); analogWrite(RL_ENB, pwm);
  digitalWrite(FR_IN1, LOW);  digitalWrite(FR_IN2, HIGH); analogWrite(FR_ENA, pwm);
  digitalWrite(RR_IN3, HIGH); digitalWrite(RR_IN4, LOW);  analogWrite(RR_ENB, pwm);
}

void driveBackward(int pwm) {
  // Originally strafeLeft: FL Bwd, RL Fwd, FR Fwd, RR Bwd
  digitalWrite(FL_IN1, LOW);  digitalWrite(FL_IN2, HIGH); analogWrite(FL_ENA, pwm);
  digitalWrite(RL_IN3, HIGH); digitalWrite(RL_IN4, LOW);  analogWrite(RL_ENB, pwm);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);  analogWrite(FR_ENA, pwm);
  digitalWrite(RR_IN3, LOW);  digitalWrite(RR_IN4, HIGH); analogWrite(RR_ENB, pwm);
}

void strafeRight(int pwm) {
  // Originally driveForward: All Wheels Fwd
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW); analogWrite(FL_ENA, pwm);
  digitalWrite(RL_IN3, HIGH); digitalWrite(RL_IN4, LOW); analogWrite(RL_ENB, pwm);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW); analogWrite(FR_ENA, pwm);
  digitalWrite(RR_IN3, HIGH); digitalWrite(RR_IN4, LOW); analogWrite(RR_ENB, pwm);
}

void strafeLeft(int pwm) {
  // Originally driveBackward: All Wheels Bwd
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH); analogWrite(FL_ENA, pwm);
  digitalWrite(RL_IN3, LOW); digitalWrite(RL_IN4, HIGH); analogWrite(RL_ENB, pwm);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH); analogWrite(FR_ENA, pwm);
  digitalWrite(RR_IN3, LOW); digitalWrite(RR_IN4, HIGH); analogWrite(RR_ENB, pwm);
}

void stopAll() {
  analogWrite(FL_ENA, 0); analogWrite(RL_ENB, 0);
  analogWrite(FR_ENA, 0); analogWrite(RR_ENB, 0);
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, LOW);
  digitalWrite(RL_IN3, LOW); digitalWrite(RL_IN4, LOW);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, LOW);
  digitalWrite(RR_IN3, LOW); digitalWrite(RR_IN4, LOW);
}

// Single high-speed encoder ISR
ISR(PCINT0_vect) {
  uint8_t currentPortB = PINB;
  uint8_t changes = currentPortB ^ lastPortB;

  if (changes & (1 << PB3)) {
    if ((currentPortB >> PB3) & 1) { 
      if ((currentPortB >> PB2) & 1) countFL--; else countFL++;
    } else { 
      if ((currentPortB >> PB2) & 1) countFL++; else countFL--;
    }
  }
  if (changes & (1 << PB1)) {
    if ((currentPortB >> PB1) & 1) {
      if ((currentPortB >> PB0) & 1) countFR++; else countFR--;
    } else {
      if ((currentPortB >> PB0) & 1) countFR--; else countFR++;
    }
  }
  if (changes & (1 << PB4)) {
    if ((currentPortB >> PB4) & 1) {
      if ((currentPortB >> PB5) & 1) countRL--; else countRL++;
    } else {
      if ((currentPortB >> PB5) & 1) countRL++; else countRL--;
    }
  }
  if (changes & (1 << PB6)) {
    if ((currentPortB >> PB6) & 1) {
      if ((currentPortB >> PB7) & 1) countRR++; else countRR--;
    } else {
      if ((currentPortB >> PB7) & 1) countRR--; else countRR++;
    }
  }
  lastPortB = currentPortB;
}