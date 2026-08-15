#include "TCS34725.h"

static bool TCS_WriteReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(TCS34725_ADDR);
    Wire.write(TCS34725_CMD | reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static bool TCS_ReadReg(uint8_t reg, uint8_t *val) {
    Wire.beginTransmission(TCS34725_ADDR);
    Wire.write(TCS34725_CMD | reg);
    if (Wire.endTransmission(false) != 0) return false;
    
    Wire.requestFrom((uint8_t)TCS34725_ADDR, (uint8_t)1);
    if (Wire.available()) {
        *val = Wire.read();
        return true;
    }
    return false;
}

bool TCS34725_Init() {
    Wire.begin();
    
    uint8_t id = 0;
    
    /* Check device ID - must be 0x44 (TCS34725) or 0x4D (TCS34727) */
    if (!TCS_ReadReg(TCS34725_REG_ID, &id)) return false;
    if (id != 0x44 && id != 0x4D) return false;
    
    /* Set integration time ~101 ms */
    if (!TCS_WriteReg(TCS34725_REG_ATIME, TCS34725_ATIME_101MS)) return false;
    
    /* Set gain 4x */
    if (!TCS_WriteReg(TCS34725_REG_CONTROL, TCS34725_GAIN_4X)) return false;
    
    /* Power ON */
    if (!TCS_WriteReg(TCS34725_REG_ENABLE, TCS34725_ENABLE_PON)) return false;
    delay(3); /* datasheet: wait >=2.4 ms after power on */
    
    /* Enable ADC */
    if (!TCS_WriteReg(TCS34725_REG_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN)) return false;
    
    delay(120); /* wait one full integration cycle (101 ms + margin) */
    
    return true;
}

bool TCS34725_ReadRaw(TCS34725_RawData *data) {
    uint8_t cmd = TCS34725_CMD | TCS34725_CMD_AUTO | TCS34725_REG_CDATAL;
    
    Wire.beginTransmission(TCS34725_ADDR);
    Wire.write(cmd);
    if (Wire.endTransmission(false) != 0) return false;
    
    Wire.requestFrom((uint8_t)TCS34725_ADDR, (uint8_t)8);
    if (Wire.available() < 8) return false;
    
    uint8_t raw[8];
    for (int i=0; i<8; i++) {
        raw[i] = Wire.read();
    }
    
    data->c = (uint16_t)((raw[1] << 8) | raw[0]);
    data->r = (uint16_t)((raw[3] << 8) | raw[2]);
    data->g = (uint16_t)((raw[5] << 8) | raw[4]);
    data->b = (uint16_t)((raw[7] << 8) | raw[6]);
    
    return true;
}

DetectedColor TCS34725_ClassifyColor(TCS34725_RawData *data) {
    if (data->c == 0) return COLOR_UNKNOWN;
    
    /* Too dark */
    if (data->c < 80) return COLOR_BLACK;
    
    float rf = (float)data->r / data->c;
    float gf = (float)data->g / data->c;
    float bf = (float)data->b / data->c;
    
    float cmax = rf;
    if (gf > cmax) cmax = gf;
    if (bf > cmax) cmax = bf;
    
    float cmin = rf;
    if (gf < cmin) cmin = gf;
    if (bf < cmin) cmin = bf;
    
    /* Neutral / white - all channels roughly equal */
    if ((cmax - cmin) < 0.08f) return COLOR_WHITE;
    
    /* Blue dominant */
    if (bf > 0.28f && rf < 0.30f) return COLOR_BLUE;
    
    /* Red dominant */
    if (rf > 0.40f && bf < 0.25f) return COLOR_RED;
    
    return COLOR_UNKNOWN;
}
