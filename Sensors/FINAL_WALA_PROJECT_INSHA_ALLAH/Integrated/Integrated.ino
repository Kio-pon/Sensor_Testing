// ---------------------------------------------------------------------------
// Dual-Motor Automation Controller (Single L298N Driver Setup)
// ---------------------------------------------------------------------------

// --- Pusher Motor Pins (Left Side of L298N) ---
const int pusherEnA = 9;  // PWM Speed Control
const int pusherIn1 = 8;  
const int pusherIn2 = 7;  

// --- Lift Motor Pins (Right Side of L298N) ---
const int liftEnB = 10;   // PWM Speed Control
const int liftIn3 = 11;  
const int liftIn4 = 12;  

// --- Sensor & Homing Pins ---
const int encA = 2;       // Must stay on Interrupt Pin
const int encB = 3;       // Must stay on Interrupt Pin
const int returnPin = 4;  // Ground to calibrate Absolute Zero

const int pusherEncA = 5; // PUSHER ENCODER A (Polled Pin)
const int pusherEncB = 6; // PUSHER ENCODER B (Polled Pin)

// --- Mechanism Speeds & Parameters ---
const long PULSES_PER_INCH = 688;
const int liftSpeed = 200;    // JGA25-370 running speed (0-255)
const int pusherSpeed = 255;  // Safe 6V limit (0-255)
const long PUSHER_PULSES_PER_REV = 166; // Calibrated for 1 exact rotation

// --- Coordinate Targets (Vertical Lift Dimensions) ---
const float POS_1_INCHES = 1.102;   // Bottom Box (2.8cm)
const float POS_2_INCHES = 4.64531; // Top Box (11.8cm)

// --- System State Variables ---
volatile long encoderPos = 0;
int currentLiftState = 1;
bool isPusherFiring = false;
bool isColumnGoingUp = true; // Tracks serpentine direction: true = Up, false = Down

enum SystemState {
  STANDBY,
  ACTIVE
};
SystemState currentState = STANDBY;

int columnCounter = 1;
int boxesProcessed = 0;
String serialBuffer = "";

// Forward declarations
bool moveLiftToCoordinate(float targetInches, bool allowAbort = true);
bool firePusher();
void stopLift();
void stopPusher();
void triggerEmergencyReset();
void countPulses();

void setup() {
  Serial.begin(9600);

  // Initialize Pusher Outputs
  pinMode(pusherEnA, OUTPUT);
  pinMode(pusherIn1, OUTPUT);
  pinMode(pusherIn2, OUTPUT);

  // Initialize Lift Outputs
  pinMode(liftEnB, OUTPUT);
  pinMode(liftIn3, OUTPUT);
  pinMode(liftIn4, OUTPUT);

  // Initialize Input Pins
  pinMode(encA, INPUT_PULLUP);
  pinMode(encB, INPUT_PULLUP);
  pinMode(pusherEncA, INPUT_PULLUP);
  pinMode(pusherEncB, INPUT_PULLUP);
  pinMode(returnPin, INPUT_PULLUP);

  // Ensure everything starts dead still
  stopPusher();
  stopLift();

  // Attach Encoder Interrupt
  attachInterrupt(digitalPinToInterrupt(encA), countPulses, RISING);

  // 1. LIFT CALIBRATION MODE
  if (digitalRead(returnPin) == LOW) {
    Serial.println("--- LIFT CALIBRATION MODE ---");
    Serial.println("Moving to Home. UNPLUG Pin 4 to set absolute zero!");
    
    digitalWrite(liftIn3, LOW);
    digitalWrite(liftIn4, HIGH);
    analogWrite(liftEnB, liftSpeed);

    while (digitalRead(returnPin) == LOW) { }
    stopLift();
    
    noInterrupts();
    encoderPos = 0;
    interrupts();
    
    Serial.println("Home Reached. Absolute Zero Set. Press Reset to start.");
    while (true);
  }

  // 2. INITIAL STARTUP POSITION
  Serial.println("System initialized. Driving Lift to Default Position 1...");
  moveLiftToCoordinate(POS_1_INCHES, false); // Non-abortable calibration move
  currentLiftState = 1;
  Serial.println("Ready. Waiting for START command...");
}

bool checkSerialAbort() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'E' || c == 'e') {
      serialBuffer = "";
      return true;
    }
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        String cmd = serialBuffer;
        cmd.trim();
        cmd.toUpperCase();
        serialBuffer = "";
        if (cmd == "END") {
          return true;
        }
      }
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 20) {
        serialBuffer = "";
      }
    }
  }
  return false;
}

void triggerEmergencyReset() {
  Serial.println("[STATUS] EMERGENCY END RECEIVED. HALTING!");
  stopPusher();
  stopLift();
  isPusherFiring = false;
  currentState = STANDBY;
  moveLiftToCoordinate(POS_1_INCHES, false); // Reset lift without abort check recursion
  currentLiftState = 1;
  Serial.println("[STATUS] Reset to STANDBY at Box 1 Position.");
}

void processBox(bool isBlue) {
  // 1. If Blue, execute pusher firing
  if (isBlue) {
    Serial.println("\n[MEGA Signal] BLUE Box Detected!");
    if (!firePusher()) {
      triggerEmergencyReset();
      return;
    }
  } else {
    Serial.println("\n[MEGA Signal] RED Box Detected (Skip Shooting)!");
  }

  // 2. Perform Lift Movement and Column logic
  if (columnCounter == 1) {
    boxesProcessed++;
    if (boxesProcessed == 1) {
      // Column 1, Box 1: Move up to Box 2
      Serial.println("Action: Moving UP to Box 2...");
      if (!moveLiftToCoordinate(POS_2_INCHES, true)) {
        triggerEmergencyReset();
        return;
      }
      currentLiftState = 2;
      Serial.println("DONE"); // Tell Mega this single box step is done
    } else if (boxesProcessed == 2) {
      // Column 1, Box 2: Column 1 is done
      Serial.println("Action: Column 1 complete. Lift staying at Top.");
      Serial.println("COLUMN 1 DONE"); // Send exact Column 1 Done message
      isColumnGoingUp = false;
    }
  } else if (columnCounter == 2) {
    boxesProcessed++;
    if (boxesProcessed == 3) {
      // Column 2, Box 2 (first box scanned in Column 2 is Box 2 because we are going down)
      Serial.println("Action: Moving DOWN to Box 1...");
      if (!moveLiftToCoordinate(POS_1_INCHES, true)) {
        triggerEmergencyReset();
        return;
      }
      currentLiftState = 1;
      Serial.println("DONE"); // Tell Mega this single box step is done
    } else if (boxesProcessed == 4) {
      // Column 2, Box 1: Column 2 is done
      Serial.println("Action: Column 2 complete. Lift staying at Bottom.");
      Serial.println("COLUMN 2 DONE"); // Send exact Column 2 Done message
      currentState = STANDBY; // Go back to Standby
    }
  }
}

void handleCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  if (cmd == "END" || cmd == "E") {
    triggerEmergencyReset();
  } else if (cmd == "START" || cmd == "A") {
    if (currentState == STANDBY) {
      Serial.println("System starting...");
      currentState = ACTIVE;
      columnCounter = 1;
      boxesProcessed = 0;
      isColumnGoingUp = true;
      moveLiftToCoordinate(POS_1_INCHES, false);
      currentLiftState = 1;
      Serial.println("STATE: ACTIVE, COLUMN 1");
    }
  } else if (cmd == "RESUME" || cmd == "R") {
    if (currentState == ACTIVE && columnCounter == 1 && boxesProcessed == 2) {
      Serial.println("Resuming for Column 2...");
      columnCounter = 2;
      moveLiftToCoordinate(POS_2_INCHES, false);
      currentLiftState = 2;
      Serial.println("DONE"); // Tell Mega it is at the top of Column 2
    }
  } else if (cmd == "F") {
    if (currentState == ACTIVE) {
      processBox(true);
    } else {
      Serial.println("Ignored BLUE box - System is in STANDBY");
    }
  } else if (cmd == "S") {
    if (currentState == ACTIVE) {
      processBox(false);
    } else {
      Serial.println("Ignored RED box - System is in STANDBY");
    }
  } else {
    Serial.print("Unknown Command: ");
    Serial.println(cmd);
  }
}

void checkSerialInput() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        handleCommand(serialBuffer);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > 20) {
        serialBuffer = "";
      }
    }
  }
}

void loop() {
  checkSerialInput();
  delay(10);
}

// --- Modular Action Functions ---

bool firePusher() {
  if (isPusherFiring) return true;
  isPusherFiring = true;
  Serial.println("[ACTION] Firing Pusher...");

  long currentPusherPulses = 0;
  int lastStateA = digitalRead(pusherEncA);
  long startTime = millis(); 

  digitalWrite(pusherIn1, HIGH);
  digitalWrite(pusherIn2, LOW);
  analogWrite(pusherEnA, pusherSpeed);

  while (currentPusherPulses < PUSHER_PULSES_PER_REV) {
    if (checkSerialAbort()) {
      triggerEmergencyReset();
      isPusherFiring = false;
      return false; 
    }

    if (millis() - startTime > 4000) {
      Serial.println("[ERROR] Pusher Timeout!");
      break;
    }

    int currentStateA = digitalRead(pusherEncA);
    if (currentStateA == HIGH && lastStateA == LOW) {
      currentPusherPulses++;
    }
    lastStateA = currentStateA; 
  }

  stopPusher();
  Serial.println("[ACTION] Pusher cycle complete.");
  isPusherFiring = false;
  return true;
}

bool moveLiftToCoordinate(float targetInches, bool allowAbort) {
  long targetPos = targetInches * PULSES_PER_INCH;
  
  noInterrupts();
  long currentPos = encoderPos;
  interrupts();

  if (currentPos == targetPos) return true;

  int direction;
  if (targetPos > currentPos) {
    direction = 1;
    digitalWrite(liftIn3, HIGH);
    digitalWrite(liftIn4, LOW);
  } else {
    direction = -1;
    digitalWrite(liftIn3, LOW);  
    digitalWrite(liftIn4, HIGH);
  }

  analogWrite(liftEnB, liftSpeed);
  long startTime = millis();

  bool success = true;
  while (true) {
    if (allowAbort && checkSerialAbort()) {
      triggerEmergencyReset();
      success = false;
      break;
    }

    if (millis() - startTime > 8000) {
      Serial.println("[ERROR] Lift Timeout!");
      success = false;
      break;
    }

    noInterrupts();
    currentPos = encoderPos;
    interrupts();

    if (direction == 1 && currentPos >= targetPos) break;
    if (direction == -1 && currentPos <= targetPos) break;
    delay(5);
  }

  stopLift();
  return success;
}

// --- Hardware Braking & Sensor Routines ---

void stopPusher() {
  digitalWrite(pusherIn1, LOW);
  digitalWrite(pusherIn2, LOW);
  analogWrite(pusherEnA, 255);
  delay(150);
  analogWrite(pusherEnA, 0);  
}

void stopLift() {
  digitalWrite(liftIn3, LOW);
  digitalWrite(liftIn4, LOW);
  analogWrite(liftEnB, 255);  
  delay(150);
  analogWrite(liftEnB, 0);    
}

void countPulses() {
  if (digitalRead(encB) == HIGH) {
    encoderPos++;
  } else {
    encoderPos--;
  }
}