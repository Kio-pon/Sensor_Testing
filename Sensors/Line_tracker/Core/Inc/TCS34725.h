#ifndef TCS34725_H
#define TCS34725_H

#include "main.h"

/* ── I2C Address ─────────────────────────────────────────────────── */
#define TCS34725_ADDR (0x29 << 1)

/* ── Register Map ────────────────────────────────────────────────── */
#define TCS34725_CMD 0x80
#define TCS34725_CMD_AUTO 0x20 // auto-increment bit

#define TCS34725_REG_ENABLE 0x00
#define TCS34725_REG_ATIME 0x01
#define TCS34725_REG_CONTROL 0x0F
#define TCS34725_REG_ID 0x12
#define TCS34725_REG_CDATAL 0x14 // C, R, G, B start here

/* ── Enable Register Bits ────────────────────────────────────────── */
#define TCS34725_ENABLE_PON 0x01 // Power ON
#define TCS34725_ENABLE_AEN 0x02 // ADC Enable

/* ── Integration Time ────────────────────────────────────────────── */
#define TCS34725_ATIME_101MS 0xD5

/* ── Gain ────────────────────────────────────────────────────────── */
#define TCS34725_GAIN_4X 0x01

/* ── Raw Data Struct ─────────────────────────────────────────────── */
typedef struct {
uint16_t c; // Clear
uint16_t r; // Red
uint16_t g; // Green
uint16_t b; // Blue
} TCS34725_RawData;

/* ── Color Classification Enum ───────────────────────────────────── */
typedef enum {
COLOR_UNKNOWN = 0,
COLOR_RED,
COLOR_BLUE,
COLOR_WHITE,
COLOR_BLACK
} DetectedColor;

/* ── Function Prototypes ─────────────────────────────────────────── */
HAL_StatusTypeDef TCS34725_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef TCS34725_ReadRaw(I2C_HandleTypeDef *hi2c, TCS34725_RawData *data);
DetectedColor TCS34725_ClassifyColor(TCS34725_RawData *data);

#endif /* TCS34725_H */
