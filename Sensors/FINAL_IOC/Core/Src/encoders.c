#include "encoders.h"

volatile int32_t enc1_count = 0;
volatile int32_t enc2_count = 0;
volatile int32_t enc3_count = 0;
volatile int32_t enc4_count = 0;

void Encoders_Init(void)
{
    enc1_count = 0;
    enc2_count = 0;
    enc3_count = 0;
    enc4_count = 0;
}

void Encoders_ResetAll(void)
{
    enc1_count = 0;
    enc2_count = 0;
    enc3_count = 0;
    enc4_count = 0;
}
