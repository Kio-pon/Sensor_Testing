#include "TCS34725.h"

/* ────────────────────────────────────────────────────────────────────
Internal helpers
──────────────────────────────────────────────────────────────────── */

static HAL_StatusTypeDef TCS_WriteReg(I2C_HandleTypeDef *hi2c,
uint8_t reg, uint8_t val)
{
uint8_t buf[2];
buf[0] = TCS34725_CMD | reg;
buf[1] = val;
return HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDR, buf, 2, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef TCS_ReadReg(I2C_HandleTypeDef *hi2c,
uint8_t reg, uint8_t *val)
{
uint8_t cmd = TCS34725_CMD | reg;
HAL_StatusTypeDef status;

status = HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDR, &cmd, 1, HAL_MAX_DELAY);
if (status != HAL_OK) return status;

return HAL_I2C_Master_Receive(hi2c, TCS34725_ADDR, val, 1, HAL_MAX_DELAY);
}

/* ────────────────────────────────────────────────────────────────────
TCS34725_Init
──────────────────────────────────────────────────────────────────── */
HAL_StatusTypeDef TCS34725_Init(I2C_HandleTypeDef *hi2c)
{
uint8_t id = 0;
HAL_StatusTypeDef status;

/* Check device ID — must be 0x44 (TCS34725) or 0x4D (TCS34727) */
status = TCS_ReadReg(hi2c, TCS34725_REG_ID, &id);
if (status != HAL_OK) return HAL_ERROR;
if (id != 0x44 && id != 0x4D) return HAL_ERROR;

/* Set integration time ~101 ms */
status = TCS_WriteReg(hi2c, TCS34725_REG_ATIME, TCS34725_ATIME_101MS);
if (status != HAL_OK) return status;

/* Set gain 4x */
status = TCS_WriteReg(hi2c, TCS34725_REG_CONTROL, TCS34725_GAIN_4X);
if (status != HAL_OK) return status;

/* Power ON */
status = TCS_WriteReg(hi2c, TCS34725_REG_ENABLE, TCS34725_ENABLE_PON);
if (status != HAL_OK) return status;
HAL_Delay(3); /* datasheet: wait ≥2.4 ms after power on */

/* Enable ADC */
status = TCS_WriteReg(hi2c, TCS34725_REG_ENABLE,
TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
if (status != HAL_OK) return status;

HAL_Delay(120); /* wait one full integration cycle (101 ms + margin) */

return HAL_OK;
}

/* ────────────────────────────────────────────────────────────────────
TCS34725_ReadRaw
Reads 8 bytes starting at CDATAL using auto-increment
Order: C_L, C_H, R_L, R_H, G_L, G_H, B_L, B_H
──────────────────────────────────────────────────────────────────── */
HAL_StatusTypeDef TCS34725_ReadRaw(I2C_HandleTypeDef *hi2c,
TCS34725_RawData *data)
{
uint8_t cmd = TCS34725_CMD | TCS34725_CMD_AUTO | TCS34725_REG_CDATAL;
uint8_t raw[8];
HAL_StatusTypeDef status;

status = HAL_I2C_Master_Transmit(hi2c, TCS34725_ADDR, &cmd, 1, HAL_MAX_DELAY);
if (status != HAL_OK) return status;

status = HAL_I2C_Master_Receive(hi2c, TCS34725_ADDR, raw, 8, HAL_MAX_DELAY);
if (status != HAL_OK) return status;

data->c = (uint16_t)((raw[1] << 8) | raw[0]);
data->r = (uint16_t)((raw[3] << 8) | raw[2]);
data->g = (uint16_t)((raw[5] << 8) | raw[4]);
data->b = (uint16_t)((raw[7] << 8) | raw[6]);

return HAL_OK;
}

/* ────────────────────────────────────────────────────────────────────
TCS34725_ClassifyColor
Same thresholds as your original Arduino code
──────────────────────────────────────────────────────────────────── */
DetectedColor TCS34725_ClassifyColor(TCS34725_RawData *data)
{
if (data->c == 0) return COLOR_UNKNOWN;

/* Too dark */
if (data->c < 80) return COLOR_BLACK;

float rf = (float)data->r / data->c;
float gf = (float)data->g / data->c;
float bf = (float)data->b / data->c;

/* Find cmax and cmin without using math.h */
float cmax = rf;
if (gf > cmax) cmax = gf;
if (bf > cmax) cmax = bf;

float cmin = rf;
if (gf < cmin) cmin = gf;
if (bf < cmin) cmin = bf;

/* Neutral / white — all channels roughly equal */
if ((cmax - cmin) < 0.08f) return COLOR_WHITE;

/* Blue dominant */
if (bf > 0.28f && rf < 0.30f) return COLOR_BLUE;

/* Red dominant */
if (rf > 0.40f && bf < 0.25f) return COLOR_RED;

return COLOR_UNKNOWN;
}
