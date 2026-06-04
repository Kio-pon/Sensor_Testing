#include "imu.h"
#include "main.h"
#include <stdio.h>

float gyro_yaw_deg = 0.0f;
float gyro_pitch_deg = 0.0f;
float gyro_roll_deg = 0.0f;

static float gyro_z_offset = 0.0f;
static float gyro_x_offset = 0.0f;
static float gyro_y_offset = 0.0f;
static uint32_t last_gyro_time = 0;

static void IMU_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data) {
    uint8_t tx[2];
    tx[0] = reg & 0x7F; // Write mode (bit 7 = 0)
    tx[1] = data;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(hspi, tx, 2, 10);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
}

static uint8_t IMU_ReadReg(SPI_HandleTypeDef *hspi, uint8_t reg) {
    uint8_t tx[2] = {0}, rx[2] = {0};
    tx[0] = reg | 0x80; // Read mode (bit 7 = 1)
    tx[1] = 0x00;
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx, rx, 2, 10);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    return rx[1];
}

void IMU_Init(SPI_HandleTypeDef *hspi) {
    printf("\r\n[IMU] Starting Initialization...\r\n");
    
    // Manually configure PE3 as Chip Select Output
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    // Manually configure SPI1 pins (PA5=SCK, PA6=MISO, PA7=MOSI) to AF5
    // since CubeMX apparently missed HAL_SPI_MspInit
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    HAL_Delay(100);
    
    uint8_t whoami = IMU_ReadReg(hspi, I3G4250D_WHO_AM_I);
    printf("[IMU] WHO_AM_I: 0x%02X\r\n", whoami);
    // Usually 0xD3 or 0xD4
    
    // CTRL_REG1: PD=1 (Normal mode), Zen=1, Yen=1, Xen=1
    // Data rate: 200Hz, Cutoff: 12.5Hz -> 0x4F
    IMU_WriteReg(hspi, I3G4250D_CTRL_REG1, 0x4F);
    
    // CTRL_REG4: FS=250 dps (0x00), BDU=1 (Block data update) -> 0x80
    IMU_WriteReg(hspi, I3G4250D_CTRL_REG4, 0x80);
    
    HAL_Delay(100);

    // Calibrate offset
    printf("[IMU] Calibrating offset (Keep robot perfectly still!)...\r\n");
    int32_t x_sum = 0, y_sum = 0, z_sum = 0;
    int samples = 200; // 400ms total calibration time
    for(int i = 0; i < samples; i++) {
        uint8_t tx[7] = {0};
        uint8_t rx[7] = {0};
        tx[0] = I3G4250D_OUT_X_L | 0xC0; 
        
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_SPI_TransmitReceive(hspi, tx, rx, 7, 10);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

        int16_t x_raw = (int16_t)((rx[2] << 8) | rx[1]);
        int16_t y_raw = (int16_t)((rx[4] << 8) | rx[3]);
        int16_t z_raw = (int16_t)((rx[6] << 8) | rx[5]);
        
        x_sum += x_raw;
        y_sum += y_raw;
        z_sum += z_raw;
        HAL_Delay(2);
    }
    gyro_x_offset = (float)x_sum / samples;
    gyro_y_offset = (float)y_sum / samples;
    gyro_z_offset = (float)z_sum / samples;
    
    printf("[IMU] Calibration Done! Z-offset: %.2f\r\n", (double)gyro_z_offset);

    last_gyro_time = HAL_GetTick();
    gyro_yaw_deg = 0.0f;
    gyro_pitch_deg = 0.0f;
    gyro_roll_deg = 0.0f;
}

void IMU_ReadGyro(SPI_HandleTypeDef *hspi) {
    uint32_t now = HAL_GetTick();
    float dt = (now - last_gyro_time) / 1000.0f;
    if (dt <= 0.0f) return;
    last_gyro_time = now;

    uint8_t tx[7] = {0};
    uint8_t rx[7] = {0};
    
    // Bit 7=1 (Read), Bit 6=1 (Auto-increment address)
    tx[0] = I3G4250D_OUT_X_L | 0xC0; 
    
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(hspi, tx, rx, 7, 10);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);

    int16_t x_raw = (int16_t)((rx[2] << 8) | rx[1]);
    int16_t y_raw = (int16_t)((rx[4] << 8) | rx[3]);
    int16_t z_raw = (int16_t)((rx[6] << 8) | rx[5]);
    
    // Apply offset
    float x_rate = (x_raw - gyro_x_offset);
    float y_rate = (y_raw - gyro_y_offset);
    float z_rate = (z_raw - gyro_z_offset);
    
    // FS=250 dps: 8.75 mdps/digit
    x_rate *= 0.00875f;
    y_rate *= 0.00875f;
    z_rate *= 0.00875f;
    
    // Apply deadband (Ignore noise below 1.5 degrees per second)
    if (x_rate < 1.5f && x_rate > -1.5f) x_rate = 0.0f;
    if (y_rate < 1.5f && y_rate > -1.5f) y_rate = 0.0f;
    if (z_rate < 1.5f && z_rate > -1.5f) z_rate = 0.0f;

    gyro_roll_deg += x_rate * dt;
    gyro_pitch_deg += y_rate * dt;
    gyro_yaw_deg += z_rate * dt;
}
