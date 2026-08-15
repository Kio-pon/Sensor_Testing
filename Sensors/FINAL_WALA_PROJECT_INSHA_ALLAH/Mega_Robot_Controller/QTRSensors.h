#ifndef QTR_SENSORS_H
#define QTR_SENSORS_H

#include <Arduino.h>

void initQTR();
void calibrateQTR();
float getLinePosition(float lastPosition);
bool isLineLost();
bool isJunction();

#endif
