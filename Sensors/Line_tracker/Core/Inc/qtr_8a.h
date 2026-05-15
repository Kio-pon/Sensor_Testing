#ifndef QTR_8A_H
#define QTR_8A_H

#include "main.h"
#include <stdbool.h>

// Define the number of sensors you are using
#define NUM_QTR_SENSORS 8

void QTR_Init(ADC_HandleTypeDef *hadc);
void QTR_Read(uint16_t *sensor_values);
uint8_t QTR_ReadDigital(void);

#endif /* QTR_8A_H */
