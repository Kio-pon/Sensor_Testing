#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- QTR-8A PINS ---
extern const uint8_t QTR_PINS[8];
#define BLACK_THRESHOLD  750 // Tuned for 10-bit ADC

// --- SHARP SENSOR ---
#define SHARP_PIN  A8
#define RACK_DETECT_THRESHOLD  400
#define SHARP_NOISE_SAMPLES    5

// --- MOTOR PINS ---
#define FL_ENA  2
#define FL_IN1  22
#define FL_IN2  23

#define RL_ENB  3
#define RL_IN3  24
#define RL_IN4  25

#define FR_ENA  4
#define FR_IN1  26
#define FR_IN2  27

#define RR_ENB  5
#define RR_IN3  28
#define RR_IN4  29

// --- DRIVE PARAMETERS ---
#define BASE_SPEED  180
#define RAMP_SPEED  255
#define MIN_PWM     60

// --- PHYSICAL CONSTANTS (Matched to STM32 Config) ---
#define WHEEL_DIAMETER_MM    80.0f
#define ENCODER_PPR          11.0f
#define GEAR_RATIO           19.7f
#define TICKS_PER_WHEEL_REV  (ENCODER_PPR * 4.0f * GEAR_RATIO) // ~866.8

#define WHEEL_CIRC_CM        (3.14159265f * WHEEL_DIAMETER_MM / 10.0f) // ~25.13
#define TICKS_PER_CM         (TICKS_PER_WHEEL_REV / WHEEL_CIRC_CM)     // ~34.5

#define BOX_CM               30.48f
#define TICKS_PER_BOX        ((long)(BOX_CM * TICKS_PER_CM))        // ~1051

#define RAMP_HYP_CM          33.02f
#define TICKS_RAMP           ((long)(RAMP_HYP_CM * TICKS_PER_CM))   // ~1139

// This is an estimated multiplier for turning in place (Yaw)
#define TICKS_PER_DEGREE     10.0f

// --- LINE FOLLOWING CONSTANTS (Matched to STM32) ---
#define LINE_CENTER          3500.0f
#define LINE_DEADBAND        30.0f

// --- ENCODER PINS (Using Mega PORTB for PCINT0_vect) ---
// These specific pins were chosen so we can read all 4 encoders with a single high-speed interrupt
#define FL_ENCODER_A 50 // PB3
#define FL_ENCODER_B 51 // PB2

#define FR_ENCODER_A 52 // PB1
#define FR_ENCODER_B 53 // PB0

#define RL_ENCODER_A 10 // PB4
#define RL_ENCODER_B 11 // PB5

#define RR_ENCODER_A 12 // PB6
#define RR_ENCODER_B 13 // PB7

#endif
