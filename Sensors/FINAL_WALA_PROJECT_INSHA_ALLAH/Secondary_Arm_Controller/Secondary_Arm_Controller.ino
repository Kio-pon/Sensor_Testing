/*
 * NERC 2026 - Second Arduino (Arm / Pallet Placement Controller)
 * 
 * Waits for "RED" or "BLUE" from the Mega.
 * - "RED": Triggers Linear Actuator.
 * - "BLUE": Triggers Shooter Mechanism.
 */

#include <Arduino.h>

// Dummy pins for mechanisms, replace with actual pins
#define ACTUATOR_PIN  5
#define SHOOTER_PIN   6

bool busy = false;

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(50); 
  
  pinMode(ACTUATOR_PIN, OUTPUT);
  pinMode(SHOOTER_PIN, OUTPUT);
  digitalWrite(ACTUATOR_PIN, LOW);
  digitalWrite(SHOOTER_PIN, LOW);
}

void triggerActuator() {
  // Replace with actual linear actuator logic (e.g., relay ON/OFF or Servo)
  digitalWrite(ACTUATOR_PIN, HIGH);
  delay(1500); // Simulate actuator moving
  digitalWrite(ACTUATOR_PIN, LOW);
}

void triggerShooter() {
  // Replace with actual shooting logic
  digitalWrite(SHOOTER_PIN, HIGH);
  delay(1000); // Simulate shot taking place
  digitalWrite(SHOOTER_PIN, LOW);
}

void loop() {
  if (Serial.available() && !busy) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "RED" || cmd == "BLUE") {
      busy = true;

      if (cmd == "RED") {
        triggerActuator();
      } else if (cmd == "BLUE") {
        triggerShooter();
      }
      
      // Notify main board that mechanism has finished
      Serial.println("DONE");
      busy = false;
    }
  }
}
