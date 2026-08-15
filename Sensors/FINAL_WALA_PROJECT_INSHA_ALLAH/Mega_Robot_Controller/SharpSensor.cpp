#include "SharpSensor.h"
#include "Config.h"

void initSharp() {
  pinMode(SHARP_PIN, INPUT);
}

bool isObstacleDetected() {
  uint8_t count = 0;
  for(int i = 0; i < SHARP_NOISE_SAMPLES; i++) {
    if(analogRead(SHARP_PIN) > RACK_DETECT_THRESHOLD) count++;
  }
  return (count >= (SHARP_NOISE_SAMPLES - 1));
}
