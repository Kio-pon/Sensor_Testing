#ifndef QTR_ARRAY_H
#define QTR_ARRAY_H

#include "stm32f3xx_hal.h"
#include <stdint.h>

extern uint16_t qtr_front[8];
extern uint16_t qtr_left[6];
extern uint16_t qtr_right[6];

/* Poll all 20 analog sensors across the 4 ADCs */
void QTR_Poll(ADC_HandleTypeDef* hadc1, ADC_HandleTypeDef* hadc2, ADC_HandleTypeDef* hadc3, ADC_HandleTypeDef* hadc4);

/* Return line position (-3500 to +3500, 0 is centered) */
int32_t QTR_GetFrontLinePosition(void);

/* Return line position (-2500 to +2500, 0 is centered) */
int32_t QTR_GetLeftLinePosition(void);
int32_t QTR_GetRightLinePosition(void);

#endif /* QTR_ARRAY_H */
