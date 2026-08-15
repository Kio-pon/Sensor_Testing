#include "QTRSensors.h"
#include "Config.h"

const uint8_t QTR_PINS[8] = {A0, A1, A2, A3, A4, A5, A6, A7};

uint16_t qtrMin[8];
uint16_t qtrMax[8];

bool _lineLost = false;
bool _isJunction = false;

bool isLineLost() {
  return _lineLost;
}

bool isJunction() {
  return _isJunction;
}

void initQTR() {
  for (int i = 0; i < 8; i++) {
    pinMode(QTR_PINS[i], INPUT);
    qtrMin[i] = 1023;
    qtrMax[i] = 0;
  }
}

void calibrateQTR() {
  for (int i = 0; i < 8; i++) {
    uint16_t val = analogRead(QTR_PINS[i]);
    if (val < qtrMin[i]) qtrMin[i] = val;
    if (val > qtrMax[i]) qtrMax[i] = val;
  }
}

uint16_t readQTR(uint8_t pin) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 4; i++) {
    sum += analogRead(pin);
  }
  return (uint16_t)(sum >> 2); 
}

// Proper ADC Line Position with Dynamic Calibration Min/Max Normalization
float getLinePosition(float lastPosition) {
  uint32_t weightedSum = 0;
  uint32_t sum = 0;
  bool onLine = false;
  uint8_t blackCount = 0; // Track how many sensors see black

  for (uint8_t i = 0; i < 8; i++) {
    uint16_t val = readQTR(QTR_PINS[i]);
    
    // Normalize reading using calibrated Min and Max arrays
    uint16_t calMin = qtrMin[i];
    uint16_t calMax = qtrMax[i];
    if (calMax <= calMin) calMax = calMin + 1; // Prevent division by zero

    val = constrain(val, calMin, calMax);
    
    // Low analog value = black line (reflection absorbed). High = white floor.
    // Calculate a 0-1000 scale where 1000 is maximum black (val == calMin)
    int32_t weight = 1000 - ((val - calMin) * 1000L / (calMax - calMin));
    
    // Threshold out the floor noise (e.g., if normalized weight > 200, it's on the line)
    if (weight > 200) {
      onLine = true;
      blackCount++;
    } else {
      weight = 0; 
    }

    weightedSum += (weight * i * 1000);
    sum += weight;
  }

  // If 5 or more sensors see black, we've hit a horizontal junction line!
  // (Reduced from 7 to 5 to make it much more sensitive to thin lines or slight angles)
  _isJunction = (blackCount >= 5);

  if (!onLine || sum == 0) {
    // Lost line fallback
    _lineLost = true;
    return (lastPosition < LINE_CENTER) ? 0.0f : 7000.0f; 
  }
  
  _lineLost = false;
  return (float)weightedSum / (float)sum;
}
