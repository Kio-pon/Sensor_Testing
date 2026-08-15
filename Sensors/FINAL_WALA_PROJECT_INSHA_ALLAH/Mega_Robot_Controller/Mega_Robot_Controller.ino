/*
 * NERC 2026 - Arduino Mega Migration (MODULAR ROBUST VERSION)
 */

#include <Arduino.h>
#include <Wire.h>
#include "Config.h"
#include "MecanumDrive.h"
#include "QTRSensors.h"
#include "SharpSensor.h"
#include "Encoders.h"
#include "TCS34725.h"

// --- PID PARAMETERS ---
// Note: Since error is now on a scale of [-3500, 3500], Kp and Kd are scaled down by 1000
// to match your previous tuning ratios! Kp 35.0 -> 0.035
float Kp = 0.035;
float Ki = 0.0;
float Kd = 0.020;

float lastError   = 0.0;
float integral    = 0.0;
float lastPosition = 3500.0f; // Track position instead of error for the QTR fallback

// --- STATE MACHINE ---
enum State {
  STATE_FOLLOW_LINE,
  STATE_DETECT_BOX,
  STATE_WAIT_FOR_ARM,
  STATE_STRAFE_LEFT_TO_NEXT_BOX,
  STATE_CLIMB_RAMP,
  STATE_PARK,
  STATE_DONE
};

State currentState = STATE_FOLLOW_LINE;
uint8_t blueBoxesFound = 0;
String detectedColor = "";

unsigned long stateTimer = 0;
#define ARM_TIMEOUT_MS 10000

// We need a specific amount of encoder ticks to strafe to the next box.
// Using exact TICKS_PER_BOX from Config.h (~1051 ticks for 30.48 cm)
#define STRAFE_ENCODER_TICKS TICKS_PER_BOX 

// Function Prototypes
void processIncomingUART();
void followLineStep();
void followLineToJunction(int targetJunctionCount);
void strafeRightForDistance(float cm);
void spinRightDegrees(float degrees);
void driveBackwardUntilLine();
void followLineBackwardUntilEnd();
void followLineUntilEnd();
void followLine();
void detectBoxAndReadColor();
void waitForArm();
void strafeToNextBox();
void climbRamp();
void park();

void setup() {
  Serial.begin(115200);   
  Serial1.begin(9600);    
  Serial1.setTimeout(50); 

  initMotors();
  initQTR();
  initSharp();
  initEncoders();

  // Initialize Color Sensor via Wire (I2C)
  if (!TCS34725_Init()) {
    Serial.println("Warning: TCS34725 Color Sensor failed to initialize!");
  }

  Serial.println("Calibrating QTR Sensors...");
  unsigned long calibStart = millis();

  // Spin Right for 1.5 seconds
  while (millis() - calibStart < 1500) {
    spinRight(120);
    calibrateQTR();
  }
  
  // Spin Left for 3.0 seconds
  calibStart = millis();
  while (millis() - calibStart < 3000) {
    spinLeft(120);
    calibrateQTR();
  }

  // Spin Right for 1.5 seconds back to center
  calibStart = millis();
  while (millis() - calibStart < 1500) {
    spinRight(120);
    calibrateQTR();
  }
  
  stopAll();
  Serial.println("Calibration complete.");

  Serial.println("Following line until the 3rd Junction...");
  followLineToJunction(3);
  
  Serial.println("Reached 3rd Junction! Strafing Right for 40cm...");
  strafeRightForDistance(40.0f);
  
  Serial.println("Spinning right 4 degrees to correct strafe drift...");
  spinRightDegrees(4.0f);

  Serial.println("Backing up to find the line...");
  driveBackwardUntilLine();

  Serial.println("Line found! Following backward until all whites are seen for 1 second...");
  followLineBackwardUntilEnd();

  Serial.println("Sequence complete. Starting main rack/box hunt sequence.");
}

void processIncomingUART() {
  if (Serial1.available()) {
    String msg = Serial1.readStringUntil('\n');
    msg.trim();
    msg.toUpperCase();
    
    if (msg == "DONE" && currentState == STATE_WAIT_FOR_ARM) {
      Serial.println("Arm sequence completed.");

      if (detectedColor == "BLUE") {
        blueBoxesFound++;
        Serial.print("Blue boxes found so far: ");
        Serial.println(blueBoxesFound);
      }

      if (blueBoxesFound >= 2) {
        Serial.println("Found 2 Blue Boxes! Moving to Ramp Phase.");
        currentState = STATE_CLIMB_RAMP;
        stateTimer = millis();
      } else {
        Serial.println("Hunting for more boxes. Strafing left.");
        resetEncoders();
        currentState = STATE_STRAFE_LEFT_TO_NEXT_BOX;
      }
    }
  }
}

void loop() {
  processIncomingUART();

  switch (currentState) {
    case STATE_FOLLOW_LINE:
      followLine();
      break;
    case STATE_DETECT_BOX:
      detectBoxAndReadColor();
      break;
    case STATE_WAIT_FOR_ARM:
      waitForArm();
      break;
    case STATE_STRAFE_LEFT_TO_NEXT_BOX:
      strafeToNextBox();
      break;
    case STATE_CLIMB_RAMP:
      climbRamp();
      break;
    case STATE_PARK:
      park();
      break;
    case STATE_DONE:
      stopAll();
      break;
  }
}

void followLineStep() {
  float position = getLinePosition(lastPosition);
  lastPosition = position;
  
  float error = position - LINE_CENTER; // Center is 3500

  // Apply deadband from Config.h
  if (abs(error) < LINE_DEADBAND) {
    error = 0;
  }

  integral += error;
  if (error == 0 || (error > 0 && lastError < 0) || (error < 0 && lastError > 0)) {
    integral = 0; 
  }
  integral = constrain(integral, -10000.0, 10000.0);

  float derivative = error - lastError;
  float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
  lastError = error;

  correction = constrain(correction, -BASE_SPEED, BASE_SPEED);

  // FIXED PID LOGIC: If line is to the right (error > 0), correction is positive.
  // To turn right, the left wheel must spin FASTER than the right wheel.
  int leftSpeed  = BASE_SPEED + (int)correction;
  int rightSpeed = BASE_SPEED - (int)correction;

  if(leftSpeed > 0 && leftSpeed < MIN_PWM) leftSpeed = MIN_PWM;
  if(rightSpeed > 0 && rightSpeed < MIN_PWM) rightSpeed = MIN_PWM;
  if(leftSpeed < 0) leftSpeed = 0;
  if(rightSpeed < 0) rightSpeed = 0;

  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  driveForward(leftSpeed, rightSpeed);
}

void followLine() {
  followLineStep();

  if (isObstacleDetected()) {
    currentState = STATE_DETECT_BOX;
  }
}

void followLineToJunction(int targetJunctionCount) {
  int junctionCount = 0;
  bool wasJunction = false;
  long lastJunctionTime = 0;
  
  while (true) {
    followLineStep();
    
    bool currentJunction = isJunction();
    
    // Debounce: ensure we don't count the same horizontal line 50 times in a row!
    if (currentJunction && !wasJunction && (millis() - lastJunctionTime > 200)) {
      junctionCount++;
      Serial.print("Junction detected! Count: ");
      Serial.println(junctionCount);
      lastJunctionTime = millis();
      
      if (junctionCount >= targetJunctionCount) {
        stopAll();
        Serial.println("Target junction reached!");
        break;
      }
    }
    wasJunction = currentJunction;
  }
}

void strafeRightForDistance(float cm) {
  long targetTicks = (long)(cm * TICKS_PER_CM);
  resetEncoders();

  while (true) {
    long avgTicks = (abs(getEncoderFL()) + abs(getEncoderFR()) + abs(getEncoderRL()) + abs(getEncoderRR())) / 4;
    
    if (avgTicks >= targetTicks) {
      stopAll();
      break;
    }
    
    strafeRight(BASE_SPEED);
  }
}

void followLineUntilEnd() {
  long whiteStartTime = 0;
  
  while (true) {
    followLineStep();
    
    if (isLineLost()) {
      if (whiteStartTime == 0) {
        whiteStartTime = millis();
      } else if (millis() - whiteStartTime >= 1000) {
        stopAll();
        Serial.println("Line ended (All Whites detected for 1 full second)!");
        break;
      }
    } else {
      whiteStartTime = 0;
    }
  }
}

void spinRightDegrees(float degrees) {
  long targetTicks = (long)(degrees * TICKS_PER_DEGREE);
  resetEncoders();

  while (true) {
    long avgTicks = (abs(getEncoderFL()) + abs(getEncoderFR()) + abs(getEncoderRL()) + abs(getEncoderRR())) / 4;
    
    if (avgTicks >= targetTicks) {
      stopAll();
      break;
    }
    
    spinRight(120);
  }
}

void driveBackwardUntilLine() {
  while (true) {
    driveBackward(BASE_SPEED, BASE_SPEED);
    getLinePosition(3500.0f); // Update QTR state internally
    
    if (!isLineLost()) {
      stopAll();
      delay(200); // physical momentum stop
      break;
    }
  }
}

void followLineBackwardUntilEnd() {
  long whiteStartTime = 0;
  
  while (true) {
    // Apply PID math to backward driving
    float position = getLinePosition(lastPosition);
    lastPosition = position;
    float error = position - LINE_CENTER;

    if (abs(error) < LINE_DEADBAND) error = 0;

    integral += error;
    if (error == 0 || (error > 0 && lastError < 0) || (error < 0 && lastError > 0)) integral = 0; 
    integral = constrain(integral, -10000.0, 10000.0);

    float derivative = error - lastError;
    float correction = (Kp * error) + (Ki * integral) + (Kd * derivative);
    lastError = error;

    correction = constrain(correction, -BASE_SPEED, BASE_SPEED);

    // To turn the FRONT right while backing up, the left wheels must back up faster than the right wheels.
    int leftSpeed  = BASE_SPEED + (int)correction;
    int rightSpeed = BASE_SPEED - (int)correction;

    if(leftSpeed > 0 && leftSpeed < MIN_PWM) leftSpeed = MIN_PWM;
    if(rightSpeed > 0 && rightSpeed < MIN_PWM) rightSpeed = MIN_PWM;
    if(leftSpeed < 0) leftSpeed = 0;
    if(rightSpeed < 0) rightSpeed = 0;

    leftSpeed  = constrain(leftSpeed,  0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    driveBackward(leftSpeed, rightSpeed);
    
    // Check for 1 second of all whites
    if (isLineLost()) {
      if (whiteStartTime == 0) {
        whiteStartTime = millis();
      } else if (millis() - whiteStartTime >= 1000) {
        stopAll();
        Serial.println("Backward Line ended (All Whites detected for 1 full second)!");
        break;
      }
    } else {
      whiteStartTime = 0;
    }
  }
}

void detectBoxAndReadColor() {
  stopAll();
  delay(200); // Allow physical momentum to stop

  // Take a color reading
  TCS34725_RawData rawData;
  if (TCS34725_ReadRaw(&rawData)) {
    DetectedColor color = TCS34725_ClassifyColor(&rawData);
    
    if (color == COLOR_RED) {
      detectedColor = "RED";
      Serial.println("Color Sensor: RED detected.");
    } else if (color == COLOR_BLUE) {
      detectedColor = "BLUE";
      Serial.println("Color Sensor: BLUE detected.");
    } else {
      // If it sees white, black, or unknown, we just default to RED so it continues checking
      // (Change this logic if you want it to ignore unknown colors)
      detectedColor = "RED"; 
      Serial.println("Color Sensor: UNKNOWN (Defaulting to RED).");
    }
  } else {
    // Failsafe
    detectedColor = "RED";
    Serial.println("Color Sensor Read Failed! (Defaulting to RED).");
  }

  // Command the arm Arduino
  Serial1.println(detectedColor);
  currentState = STATE_WAIT_FOR_ARM;
  stateTimer = millis();
}

void waitForArm() {
  // If the secondary Arduino fails to reply DONE within 10 seconds, retry
  if (millis() - stateTimer > ARM_TIMEOUT_MS) {
    Serial.println("Error: Arm timeout! Retrying color command.");
    Serial1.println(detectedColor);
    stateTimer = millis(); 
  }
}

void strafeToNextBox() {
  // We use the absolute average of all 4 encoders.
  // When strafing left, FL/RR turn backwards, FR/RL turn forwards (or vice versa).
  long avgTicks = (abs(getEncoderFL()) + abs(getEncoderFR()) + abs(getEncoderRL()) + abs(getEncoderRR())) / 4;

  if (avgTicks < STRAFE_ENCODER_TICKS) {
    strafeLeft(BASE_SPEED);
  } else {
    // Reached target distance
    stopAll();
    delay(200);
    
    // Resume hunting for the line / moving forward slightly to clear the current box view
    driveForward(BASE_SPEED, BASE_SPEED);
    delay(500); // clear the box temporarily
    currentState = STATE_FOLLOW_LINE;
  }
}

#define RAMP_CLIMB_MS  3000
void climbRamp() {
  driveForward(RAMP_SPEED, RAMP_SPEED);
  if (millis() - stateTimer > RAMP_CLIMB_MS) {
    stopAll();
    currentState = STATE_PARK;
    stateTimer = millis();
  }
}

#define PARK_MS  1500
void park() {
  driveForward(BASE_SPEED, BASE_SPEED);
  if (millis() - stateTimer > PARK_MS) {
    stopAll();
    currentState = STATE_DONE;
    Serial.println("PARKED");
  }
}
