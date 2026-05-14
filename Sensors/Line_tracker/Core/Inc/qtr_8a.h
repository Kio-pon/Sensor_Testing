#ifndef QTR_8A_H
#define QTR_8A_H

#include "main.h"

// Define the number of sensors you are using
#define NUM_QTR_SENSORS 4

// Function prototypes
void QTR_Init(ADC_HandleTypeDef *hadc);
void QTR_Read(uint16_t *sensor_values);

#endif /* QTR_8A_H */
