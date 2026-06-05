#ifndef SHARP_IR_H
#define SHARP_IR_H

#include "main.h"

extern uint16_t sharp_ir_raw;

void Sharp_Poll(ADC_HandleTypeDef* hadc);
float Sharp_GetDistanceCM(void);

#endif // SHARP_IR_H
