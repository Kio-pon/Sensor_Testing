#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile int32_t enc1_count;
extern volatile int32_t enc2_count;
extern volatile int32_t enc3_count;
extern volatile int32_t enc4_count;

void Encoders_Init(void);
void Encoders_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif /* ENCODERS_H */
