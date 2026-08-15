#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

void initEncoders();
void resetEncoders();

long getEncoderFL();
long getEncoderFR();
long getEncoderRL();
long getEncoderRR();

#endif
