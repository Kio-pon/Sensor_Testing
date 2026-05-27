/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_driver.h"
#include "qtr_8a.h"
#include "stm32f303xc.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */
UART_HandleTypeDef huart1;
Mecanum_Chassis_t chassis;

// Encoder counts - volatile prevents compiler optimising away in hot loop
volatile int32_t enc1_count = 0; // Encoder 1: Motor FR  (PA9=A, PA10=B)
volatile int32_t enc2_count = 0; // Encoder 2: Motor RR  (PA8=A, PC9=B)
volatile int32_t enc3_count = 0; // Encoder 3: Motor RL  (PC8=A, PC7=B)
volatile int32_t enc4_count = 0; // Encoder 4: Motor FL  (PC6=A, PD15=B)
QTR_Array_t qtr_front;
QTR_Array_t qtr_left;
QTR_Array_t qtr_right;

// High-speed ADC1 DMA Ping-Pong buffer (8 channels * 2)
#define SENSOR1_CHANNELS 8
volatile uint16_t adc1_dma_buffer[SENSOR1_CHANNELS * 2] = {0};
volatile uint16_t sensor1_filtered[SENSOR1_CHANNELS] = {0};
const float EMA_ALPHA = 0.25f;

// ── Quadrature decoder lookup table ─────────────────────────────────────
// Indexed by [prev_2bit_state][curr_2bit_state].
// bit1 = Phase A,  bit0 = Phase B
// Returns +1 (forward tick), -1 (reverse tick), 0 (no change / error)
static const int8_t Q_TABLE[4][4] = {
    { 0, -1,  1,  0 },  // prev = 00
    { 1,  0,  0, -1 },  // prev = 01
    {-1,  0,  0,  1 },  // prev = 10
    { 0,  1, -1,  0 }   // prev = 11
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_ADC4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void MX_USART1_UART_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

// Trapezoidal Deceleration: Smoothly ramps down motor speeds to 0 to protect power supplies
void Chassis_SmoothStop(Mecanum_Chassis_t *ch, int32_t start_vx, int32_t start_vy, int32_t start_omega, uint32_t duration_ms) {
    uint32_t steps = 6;
    uint32_t step_delay = duration_ms / steps;
    for (int i = (int)steps; i >= 0; i--) {
        int32_t vx = (start_vx * i) / (int32_t)steps;
        int32_t vy = (start_vy * i) / (int32_t)steps;
        int32_t omega = (start_omega * i) / (int32_t)steps;
        Chassis_Drive(ch, vx, vy, omega);
        HAL_Delay(step_delay);
    }
    Chassis_Drive(ch, 0, 0, 0);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  MX_USART1_UART_Init();
  printf("\r\n==================================\r\n");
  printf("  NERC ROBOT EARLY BOOT DIAGNOSTIC\r\n");
  printf("==================================\r\n");
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_ADC4_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USB_PCD_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("INIT: CONFIGURING MOTOR CHASSIS & SENSORS...\r\n");

  // Front-Right Motor (M1 = Top Right)
  chassis.fr.IN1_Port = GPIOD;
  chassis.fr.IN1_Pin  = GPIO_PIN_2;
  chassis.fr.IN2_Port = GPIOD;
  chassis.fr.IN2_Pin  = GPIO_PIN_1;
  chassis.fr.htim     = &htim2;
  chassis.fr.channel  = TIM_CHANNEL_1;
  chassis.fr.max_pwm  = 4800;
  
  // Rear-Right Motor (M2 = Bottom Right)
  chassis.rr.IN1_Port = GPIOD;
  chassis.rr.IN1_Pin  = GPIO_PIN_0;
  chassis.rr.IN2_Port = GPIOC;
  chassis.rr.IN2_Pin  = GPIO_PIN_12;
  chassis.rr.htim     = &htim2;
  chassis.rr.channel  = TIM_CHANNEL_2;
  chassis.rr.max_pwm  = 4800;
  
  // Rear-Left Motor (M3 = Bottom Left)
  chassis.rl.IN1_Port = GPIOB;
  chassis.rl.IN1_Pin  = GPIO_PIN_14;
  chassis.rl.IN2_Port = GPIOB;
  chassis.rl.IN2_Pin  = GPIO_PIN_15;
  chassis.rl.htim     = &htim2;
  chassis.rl.channel  = TIM_CHANNEL_3;
  chassis.rl.max_pwm  = 4800;
  
  // Front-Left Motor (M4 = Top Left)
  chassis.fl.IN1_Port = GPIOD;
  chassis.fl.IN1_Pin  = GPIO_PIN_8;
  chassis.fl.IN2_Port = GPIOD;
  chassis.fl.IN2_Pin  = GPIO_PIN_9;
  chassis.fl.htim     = &htim2;
  chassis.fl.channel  = TIM_CHANNEL_4;
  chassis.fl.max_pwm  = 4800;
  
  // Motor driver Enable/Standby Pin
  chassis.STBY_Port   = GPIOD;
  chassis.STBY_Pin    = GPIO_PIN_5;
  
  printf("DEBUG: Starting Chassis_Init...\r\n");
  Chassis_Init(&chassis);
  printf("DEBUG: Chassis_Init completed successfully!\r\n");

  // Front QTR line array (8 channels, all on ADC1)
  ADC_HandleTypeDef *qtr_front_adcs[8] = {
      &hadc1, &hadc1, &hadc1, &hadc1, &hadc1, &hadc1, &hadc1, &hadc1
  };
  uint32_t qtr_front_ch[8] = {
      ADC_CHANNEL_2,  // PA1
      ADC_CHANNEL_3,  // PA2
      ADC_CHANNEL_4,  // PA3
      ADC_CHANNEL_6,  // PC0
      ADC_CHANNEL_7,  // PC1
      ADC_CHANNEL_8,  // PC2
      ADC_CHANNEL_9,  // PC3
      ADC_CHANNEL_5   // PF4
  };
  printf("DEBUG: Starting QTR_Init for FRONT array...\r\n");
  QTR_Init(&qtr_front, qtr_front_adcs, qtr_front_ch, 8, 500); // 500 = 50% black in calibrated [0, 1000] scale
  printf("DEBUG: FRONT QTR_Init completed successfully!\r\n");

  // Left QTR turn counter (6 channels, all on ADC3)
  ADC_HandleTypeDef *qtr_left_adcs[6] = {
      &hadc3, &hadc3, &hadc3, &hadc3, &hadc3, &hadc3
  };
  uint32_t qtr_left_ch[6] = {
      ADC_CHANNEL_1,  // PB1
      ADC_CHANNEL_2,  // PE9
      ADC_CHANNEL_3,  // PE13
      ADC_CHANNEL_5,  // PB13
      ADC_CHANNEL_6,  // PE8
      ADC_CHANNEL_7   // PD10
  };
  printf("DEBUG: Starting QTR_Init for LEFT array...\r\n");
  QTR_Init(&qtr_left, qtr_left_adcs, qtr_left_ch, 6, 500);
  printf("DEBUG: LEFT QTR_Init completed successfully!\r\n");

  // Right QTR turn counter (6 channels, D1-D5 on ADC2, D6 on ADC3)
  ADC_HandleTypeDef *qtr_right_adcs[6] = {
      &hadc2, &hadc2, &hadc2, &hadc2, &hadc2, &hadc3
  };
  uint32_t qtr_right_ch[6] = {
      ADC_CHANNEL_1,  // PA4
      ADC_CHANNEL_2,  // PA5
      ADC_CHANNEL_3,  // PA6
      ADC_CHANNEL_4,  // PA7
      ADC_CHANNEL_5,  // PC4
      ADC_CHANNEL_8   // PD11
  };
  printf("DEBUG: Starting QTR_Init for RIGHT array...\r\n");
  QTR_Init(&qtr_right, qtr_right_adcs, qtr_right_ch, 6, 500);
  printf("DEBUG: RIGHT QTR_Init completed successfully!\r\n");

  // Disable high-frequency DMA interrupts to prevent CPU starvation.
  // Hardware circular DMA will still run in the background perfectly.
  printf("DEBUG: Disabling DMA1_Channel1_IRQn interrupt in NVIC...\r\n");
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  
  // Start continuous circular DMA conversion for ADC1
  printf("DEBUG: Starting HAL_ADC_Start_DMA...\r\n");
  HAL_StatusTypeDef dma_status = HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_dma_buffer, SENSOR1_CHANNELS * 2);
  printf("DEBUG: HAL_ADC_Start_DMA completed with status: %d\r\n", (int)dma_status);

  // Configure ALL 8 user LEDs (PE8 to PE15) as push-pull outputs
  GPIO_InitTypeDef GPIO_InitStructLED = {0};
  GPIO_InitStructLED.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | 
                           GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStructLED.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructLED.Pull = GPIO_NOPULL;
  GPIO_InitStructLED.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStructLED);

  // Perform autonomous sweep calibration for all 3 sensors using our 20% speed sideways strafe sweep!
  QTR_CalibrateAllThree(&qtr_front, &qtr_left, &qtr_right, adc1_dma_buffer, 10000, &chassis);

#if 0
  // ── 4-LEG ENCODER-BASED MOTION TEST SEQUENCE ──────────────────────────────
  {
      printf("\r\n====================================================\r\n");
      printf("      STARTING 4-LEG ENCODER MOTION TEST SUITE      \r\n");
      printf("====================================================\r\n");
      printf("One tire length = 866 ticks. Target 3 lengths = 2598 ticks.\r\n");
      printf("Countdown 3 seconds...\r\n");
      HAL_Delay(1000);
      printf("2...\r\n");
      HAL_Delay(1000);
      printf("1...\r\n");
      HAL_Delay(1000);

      // Define standard C absolute value macro locally
      #define ABS(x) ((x) < 0 ? -(x) : (x))

      // High-speed encoder polling helper macro to keep pins updated in loop
      #define POLL_ENCODERS() do { \
          uint8_t curr_enc1 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)  << 1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); \
          if (curr_enc1 != local_prev_enc1) { enc1_count += Q_TABLE[local_prev_enc1][curr_enc1]; local_prev_enc1 = curr_enc1; } \
          uint8_t curr_enc2 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)  << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9); \
          if (curr_enc2 != local_prev_enc2) { enc2_count -= Q_TABLE[local_prev_enc2][curr_enc2]; local_prev_enc2 = curr_enc2; } \
          uint8_t curr_enc3 = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)  << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7); \
          if (curr_enc3 != local_prev_enc3) { enc3_count -= Q_TABLE[local_prev_enc3][curr_enc3]; local_prev_enc3 = curr_enc3; } \
          uint8_t curr_enc4 = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)  << 1) | HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15); \
          if (curr_enc4 != local_prev_enc4) { enc4_count += Q_TABLE[local_prev_enc4][curr_enc4]; local_prev_enc4 = curr_enc4; } \
      } while(0)

      uint8_t local_prev_enc1 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)  << 1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
      uint8_t local_prev_enc2 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)  << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
      uint8_t local_prev_enc3 = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)  << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
      uint8_t local_prev_enc4 = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)  << 1) | HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);

      // --- LEG 1: FORWARD (3 tire lengths = 2598 ticks) ---
      printf("\r\n[LEG 1/4] Driving FORWARD at 100%% speed...\r\n");
      enc1_count = 0; enc2_count = 0; enc3_count = 0; enc4_count = 0;
      Chassis_Drive(&chassis, 4800, 0, 0);
      uint32_t last_print = HAL_GetTick();
      while (1) {
          POLL_ENCODERS();
          int32_t traveled = (ABS(enc1_count) + ABS(enc2_count) + ABS(enc3_count) + ABS(enc4_count)) / 4;
          uint32_t now = HAL_GetTick();
          if (now - last_print >= 100) {
              printf("  Ticks: FL=%ld, FR=%ld, RL=%ld, RR=%ld | Avg Abs Ticks: %ld / 2598\r\n",
                     (long)enc4_count, (long)enc1_count, (long)enc3_count, (long)enc2_count, (long)traveled);
              last_print = now;
          }
          if (traveled >= 2598) break;
      }
      Chassis_BrakeAll(&chassis);
      printf("  Braked! Leg 1 complete.\r\n");
      HAL_Delay(500); // 0.5s settling time

      // --- LEG 2: BACKWARD (3 tire lengths = 2598 ticks) ---
      printf("\r\n[LEG 2/4] Driving BACKWARD at 100%% speed...\r\n");
      enc1_count = 0; enc2_count = 0; enc3_count = 0; enc4_count = 0;
      Chassis_Drive(&chassis, -4800, 0, 0);
      last_print = HAL_GetTick();
      while (1) {
          POLL_ENCODERS();
          int32_t traveled = (ABS(enc1_count) + ABS(enc2_count) + ABS(enc3_count) + ABS(enc4_count)) / 4;
          uint32_t now = HAL_GetTick();
          if (now - last_print >= 100) {
              printf("  Ticks: FL=%ld, FR=%ld, RL=%ld, RR=%ld | Avg Abs Ticks: %ld / 2598\r\n",
                     (long)enc4_count, (long)enc1_count, (long)enc3_count, (long)enc2_count, (long)traveled);
              last_print = now;
          }
          if (traveled >= 2598) break;
      }
      Chassis_BrakeAll(&chassis);
      printf("  Braked! Leg 2 complete.\r\n");
      HAL_Delay(500);

      // --- LEG 3: RIGHT (3 tire lengths = 2598 ticks) ---
      printf("\r\n[LEG 3/4] Strafing RIGHT at 100%% speed...\r\n");
      enc1_count = 0; enc2_count = 0; enc3_count = 0; enc4_count = 0;
      Chassis_Drive(&chassis, 0, 4800, 0);
      last_print = HAL_GetTick();
      while (1) {
          POLL_ENCODERS();
          int32_t traveled = (ABS(enc1_count) + ABS(enc2_count) + ABS(enc3_count) + ABS(enc4_count)) / 4;
          uint32_t now = HAL_GetTick();
          if (now - last_print >= 100) {
              printf("  Ticks: FL=%ld, FR=%ld, RL=%ld, RR=%ld | Avg Abs Ticks: %ld / 2598\r\n",
                     (long)enc4_count, (long)enc1_count, (long)enc3_count, (long)enc2_count, (long)traveled);
              last_print = now;
          }
          if (traveled >= 2598) break;
      }
      Chassis_BrakeAll(&chassis);
      printf("  Braked! Leg 3 complete.\r\n");
      HAL_Delay(500);

      // --- LEG 4: LEFT (3 tire lengths = 2598 ticks) ---
      printf("\r\n[LEG 4/4] Strafing LEFT at 100%% speed...\r\n");
      enc1_count = 0; enc2_count = 0; enc3_count = 0; enc4_count = 0;
      Chassis_Drive(&chassis, 0, -4800, 0);
      last_print = HAL_GetTick();
      while (1) {
          POLL_ENCODERS();
          int32_t traveled = (ABS(enc1_count) + ABS(enc2_count) + ABS(enc3_count) + ABS(enc4_count)) / 4;
          uint32_t now = HAL_GetTick();
          if (now - last_print >= 100) {
              printf("  Ticks: FL=%ld, FR=%ld, RL=%ld, RR=%ld | Avg Abs Ticks: %ld / 2598\r\n",
                     (long)enc4_count, (long)enc1_count, (long)enc3_count, (long)enc2_count, (long)traveled);
              last_print = now;
          }
          if (traveled >= 2598) break;
      }
      Chassis_BrakeAll(&chassis);
      printf("  Braked! Leg 4 complete.\r\n");
      HAL_Delay(500);

      printf("\r\n====================================================\r\n");
      printf("     ALL 4 LEGS COMPLETED! ENTERING STATIC MONITOR   \r\n");
      printf("====================================================\r\n");
      HAL_Delay(2000);
  }
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */



  // Read initial pin states so we never count a false tick on first iteration
  uint8_t prev_enc1 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)  << 1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
  uint8_t prev_enc2 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)  << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
  uint8_t prev_enc3 = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)  << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
  uint8_t prev_enc4 = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)  << 1) | HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);

  uint32_t start_time = HAL_GetTick();
  int motor_test_state = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_time = HAL_GetTick();

    // ── QTR SENSOR EMA FILTER POLLING (1 kHz) ───────────────────────────────
    // Runs every 1ms to read from the continuous hardware circular DMA buffer.
    static uint32_t last_adc_time = 0;
    if (current_time - last_adc_time >= 1) {
        uint16_t calibrated[SENSOR1_CHANNELS];
        QTR_ReadCalibrated(&qtr_front, calibrated, adc1_dma_buffer);
        for (int i = 0; i < SENSOR1_CHANNELS; i++) {
            sensor1_filtered[i] = (uint16_t)((EMA_ALPHA * calibrated[i]) + ((1.0f - EMA_ALPHA) * sensor1_filtered[i]));
        }
        last_adc_time = current_time;
    }

    uint32_t elapsed = current_time - start_time;

    // ── 1. HIGH-SPEED ENCODER POLLING ───────────────────────────────────────
    // Runs every loop iteration (100 kHz+). Cost: ~30 CPU cycles total.

    // Encoder 1 – FR motor: Phase A = PA9, Phase B = PA10
    uint8_t curr_enc1 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9)  << 1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
    if (curr_enc1 != prev_enc1) { enc1_count += Q_TABLE[prev_enc1][curr_enc1]; prev_enc1 = curr_enc1; }

    // Encoder 2 – RR motor: Phase A = PA8, Phase B = PC9
    // Note: Swapped phase pins in wiring. Inverting count direction to read positive when moving forward.
    uint8_t curr_enc2 = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8)  << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9);
    if (curr_enc2 != prev_enc2) { enc2_count -= Q_TABLE[prev_enc2][curr_enc2]; prev_enc2 = curr_enc2; }

    // Encoder 3 – RL motor: Phase A = PC8, Phase B = PC7
    // Note: Swapped phase pins in wiring. Inverting count direction to read positive when moving forward.
    uint8_t curr_enc3 = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)  << 1) | HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7);
    if (curr_enc3 != prev_enc3) { enc3_count -= Q_TABLE[prev_enc3][curr_enc3]; prev_enc3 = curr_enc3; }

    // Encoder 4 – FL motor: Phase A = PC6, Phase B = PD15
    uint8_t curr_enc4 = (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)  << 1) | HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15);
    if (curr_enc4 != prev_enc4) { enc4_count += Q_TABLE[prev_enc4][curr_enc4]; prev_enc4 = curr_enc4; }

    // ── 2. DYNAMIC LINE-FOLLOWING CONTROLLER ─────────────────────────────────
    // Read the line position (value ranges from 0 to 7000, center is 3500)
    float position = QTR_GetLinePosition(&qtr_front, (uint16_t*)sensor1_filtered);
    
    // Calculate alignment error
    float error = 3500.0f - position;
    
    // Base forward speed set to 50% custom scale (960 compare value)
    int32_t base_speed = 1200;

    // Proportional gain - keep low to avoid overcorrection and oscillation.
    static const float Kp = 0.6f;

    // Dead-band: if the robot is within ±200 of centre (3300-3700), drive dead straight.
    // This stops the controller oscillating when it is already well-centred.
    int32_t omega;
    if (error > -200.0f && error < 200.0f) {
        omega = 0;  // Already centred - no correction needed
    } else {
        omega = (int32_t)(Kp * error);
        // Cap omega so it never exceeds base_speed (prevents spinning in place)
        if (omega >  base_speed) omega =  base_speed;
        if (omega < -base_speed) omega = -base_speed;
    }
    
    // --- 90-Degree Turn Detection ---
    // S0, S1 are leftmost sensors; S6, S7 are rightmost sensors.
    // If they are strongly black, it's a 90-degree turn.
    uint8_t left_90 = (sensor1_filtered[0] >= 750 && sensor1_filtered[1] >= 750);
    uint8_t right_90 = (sensor1_filtered[6] >= 750 && sensor1_filtered[7] >= 750);
    
    // Check if the robot has lost the line (all 8 sensors detect white / less than 300)
    uint8_t all_white = 1;
    for (int i = 0; i < 8; i++) {
        if (sensor1_filtered[i] >= 300) {
            all_white = 0; // Black line detected!
            break;
        }
    }
    
    if (left_90) {
        printf(">>> 90-DEGREE LEFT TURN DETECTED! Ramping down...\r\n");
        Chassis_SmoothStop(&chassis, base_speed, 0, omega, 180);
        HAL_Delay(150);
        
        printf(">>> Strafing Left with Active Correction...\r\n");
        // Start strafing left
        Chassis_Drive(&chassis, 0, -750, 0);
        HAL_Delay(350); // Get off the current line junction
        
        // Keep strafing left until the middle sensors re-detect the new line
        uint32_t rotate_start = HAL_GetTick();
        int32_t current_vx = 0;
        while (HAL_GetTick() - rotate_start < 4000) { // 4s safety timeout
            uint16_t calibrated[SENSOR1_CHANNELS];
            QTR_ReadCalibrated(&qtr_front, calibrated, adc1_dma_buffer);
            for (int i = 0; i < SENSOR1_CHANNELS; i++) {
                sensor1_filtered[i] = (uint16_t)((EMA_ALPHA * calibrated[i]) + ((1.0f - EMA_ALPHA) * sensor1_filtered[i]));
            }
            
            // Proportional forward/backward correction to stay centered on line while strafing
            float position = QTR_GetLinePosition(&qtr_front, (uint16_t*)sensor1_filtered);
            float error = 3500.0f - position;
            int32_t vx_correction = (int32_t)(Kp * error);
            if (vx_correction > 400) vx_correction = 400;
            if (vx_correction < -400) vx_correction = -400;
            current_vx = vx_correction;
            
            // Keep strafing left with active vx centering correction
            Chassis_Drive(&chassis, vx_correction, -750, 0);
            
            // Check if middle sensors (S3 or S4) detect the new parallel line
            if (sensor1_filtered[3] >= 550 || sensor1_filtered[4] >= 550) {
                break; // Target line centered
            }
            HAL_Delay(5);
        }
        
        // Smooth stop from the strafing motion
        Chassis_SmoothStop(&chassis, current_vx, -750, 0, 180);
        HAL_Delay(150);
    } 
    else if (right_90) {
        printf(">>> 90-DEGREE RIGHT TURN DETECTED! Ramping down...\r\n");
        Chassis_SmoothStop(&chassis, base_speed, 0, omega, 180);
        HAL_Delay(150);
        
        printf(">>> Strafing Right with Active Correction...\r\n");
        // Start strafing right
        Chassis_Drive(&chassis, 0, 750, 0);
        HAL_Delay(350); // Get off the current line junction
        
        // Keep strafing right until the middle sensors re-detect the new line
        uint32_t rotate_start = HAL_GetTick();
        int32_t current_vx = 0;
        while (HAL_GetTick() - rotate_start < 4000) { // 4s safety timeout
            uint16_t calibrated[SENSOR1_CHANNELS];
            QTR_ReadCalibrated(&qtr_front, calibrated, adc1_dma_buffer);
            for (int i = 0; i < SENSOR1_CHANNELS; i++) {
                sensor1_filtered[i] = (uint16_t)((EMA_ALPHA * calibrated[i]) + ((1.0f - EMA_ALPHA) * sensor1_filtered[i]));
            }
            
            // Proportional forward/backward correction to stay centered on line while strafing
            float position = QTR_GetLinePosition(&qtr_front, (uint16_t*)sensor1_filtered);
            float error = 3500.0f - position;
            int32_t vx_correction = (int32_t)(Kp * error);
            if (vx_correction > 400) vx_correction = 400;
            if (vx_correction < -400) vx_correction = -400;
            current_vx = vx_correction;
            
            // Keep strafing right with active vx centering correction
            Chassis_Drive(&chassis, vx_correction, 750, 0);
            
            // Check if middle sensors (S3 or S4) detect the new parallel line
            if (sensor1_filtered[3] >= 550 || sensor1_filtered[4] >= 550) {
                break; // Target line centered
            }
            HAL_Delay(5);
        }
        
        // Smooth stop from the strafing motion
        Chassis_SmoothStop(&chassis, current_vx, 750, 0, 180);
        HAL_Delay(150);
    }
    else if (all_white) {
        // Stop the car immediately (short brake) to prevent runaway off-line
        Chassis_Drive(&chassis, 0, 0, 0);
    } else {
        // Drive the robot dynamically using base forward speed and responsive turning correction
        Chassis_Drive(&chassis, base_speed, 0, omega);
    }

    // Calculate individual commanded motor speeds
    int32_t fl_cmd = base_speed - omega;
    int32_t fr_cmd = base_speed + omega;
    int32_t rl_cmd = base_speed - omega;
    int32_t rr_cmd = base_speed + omega;

    // Helper macro to calculate physical scaled PWM (50% stiction bypass)
    #define GET_PHYSICAL_PWM(s) ((s) > 0 ? (240000 + 50 * (s)) / 100 : ((s) < 0 ? -((240000 + 50 * (-(s))) / 100) : 0))

    int32_t fl_phys = GET_PHYSICAL_PWM(fl_cmd);
    int32_t fr_phys = GET_PHYSICAL_PWM(fr_cmd);
    int32_t rl_phys = GET_PHYSICAL_PWM(rl_cmd);
    int32_t rr_phys = GET_PHYSICAL_PWM(rr_cmd);

    // Map the 8 QTR sensors to the 8 Discovery LEDs (PE8 to PE15)
    // If a sensor detects the black line (value >= 500), light up its corresponding LED!
    uint16_t led_mask = 0;
    for (int i = 0; i < 8; i++) {
        if (sensor1_filtered[i] >= 500) {
            led_mask |= (1 << (8 + i)); // PE8 starts at bit 8
        }
    }
    
    // Set user LED states atomically on GPIOE (Port E)
    GPIOE->ODR = (GPIOE->ODR & ~0xFF00) | led_mask;

    // ── 3. LED HEARTBEAT & DIAGNOSTIC PRINTF ─────────────────────────────────
    // Toggle diagnostic dashboard print every 200ms
    static uint32_t last_led_time = 0;
    if (current_time - last_led_time >= 200) {
        // Clear the screen completely once when launching the dashboard to wipe
        // old prints, then use top-left overwrite for zero-flicker live updates!
        static uint8_t first_dashboard_draw = 1;
        if (first_dashboard_draw) {
            printf("\033[2J\033[H");
            first_dashboard_draw = 0;
        } else {
            printf("\033[H");
        }
        
        printf("====================================================\r\n");
        printf("          NERC ROBOT DYNAMIC LINE-FOLLOWING SUITE    \r\n");
        printf("====================================================\r\n");
        printf("SYSTEM STATUS   : %s                                \r\n", all_white ? "OFF-LINE (SAFE STOPPED)" : "ACTIVE & LINE-TRACKING");
        printf("STICTION BYPASS : 50%% MINIMUM DUTY CYCLE            \r\n");
        printf("BASE SPEED      : %d (15%% Custom Scale)            \r\n", (int)base_speed);
        printf("----------------------------------------------------\r\n");
        printf(" LINE POSITION  : %6.1f / 7000 (3500 is Center)     \r\n", (double)position);
        printf(" ALIGNMENT ERROR: %6.1f                            \r\n", (double)error);
        printf(" TURN COMMAND   : %6ld (omega)                      \r\n", (long)omega);
        printf("----------------------------------------------------\r\n");
        printf(" INDIVIDUAL MOTOR SPEEDS (COMMANDED vs PHYSICAL):   \r\n");
        printf("   FL Motor: Cmd=%4ld | Phys=%4ld (%2ld%% PWM)       \r\n", (long)fl_cmd, (long)fl_phys, (long)(fl_phys * 100 / 4800));
        printf("   FR Motor: Cmd=%4ld | Phys=%4ld (%2ld%% PWM)       \r\n", (long)fr_cmd, (long)fr_phys, (long)(fr_phys * 100 / 4800));
        printf("   RL Motor: Cmd=%4ld | Phys=%4ld (%2ld%% PWM)       \r\n", (long)rl_cmd, (long)rl_phys, (long)(rl_phys * 100 / 4800));
        printf("   RR Motor: Cmd=%4ld | Phys=%4ld (%2ld%% PWM)       \r\n", (long)rr_cmd, (long)rr_phys, (long)(rr_phys * 100 / 4800));
        printf("----------------------------------------------------\r\n");
        printf(" VISUAL REFLECTANCE MAP (LEDS PE8-PE15):            \r\n");
        printf("   [");
        for (int i = 0; i < 8; i++) {
            if (sensor1_filtered[i] >= 500) printf("*");
            else printf(" ");
        }
        printf("]  (Active Sens: ");
        for (int i = 0; i < 8; i++) {
            printf("%d ", (int)(sensor1_filtered[i] / 100));
        }
        printf(") \r\n");
        printf("----------------------------------------------------\r\n");
        printf(" ENCODER FEEDBACK (TICKS):                          \r\n");
        printf("   FL: %6ld  |  FR: %6ld                           \r\n", (long)enc4_count, (long)enc1_count);
        printf("   RL: %6ld  |  RR: %6ld                           \r\n", (long)enc3_count, (long)enc2_count);
        printf("====================================================\r\n");
        
        last_led_time = current_time;
    }

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief ADC4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC4_Init(void)
{

  /* USER CODE BEGIN ADC4_Init 0 */

  /* USER CODE END ADC4_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC4_Init 1 */

  /* USER CODE END ADC4_Init 1 */

  /** Common config
  */
  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.DiscontinuousConvMode = DISABLE;
  hadc4.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.LowPowerAutoWait = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC4_Init 2 */

  /* USER CODE END ADC4_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LD5_Pin|LD7_Pin|LD9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : MEMS_INT3_Pin MEMS_INT4_Pin MEMS_INT1_Pin MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT3_Pin|MEMS_INT4_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LD5_Pin LD7_Pin LD9_Pin */
  GPIO_InitStruct.Pin = LD5_Pin|LD7_Pin|LD9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB15 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD0 PD1
                           PD2 PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_0|GPIO_PIN_1
                          |GPIO_PIN_2|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PF6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        // Process first half: indices 0 to 7
        for (int i = 0; i < SENSOR1_CHANNELS; i++) {
            uint16_t raw = adc1_dma_buffer[i];
            sensor1_filtered[i] = (uint16_t)((EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * sensor1_filtered[i]));
        }
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        // Process second half: indices 8 to 15
        for (int i = 0; i < SENSOR1_CHANNELS; i++) {
            uint16_t raw = adc1_dma_buffer[SENSOR1_CHANNELS + i];
            sensor1_filtered[i] = (uint16_t)((EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * sensor1_filtered[i]));
        }
    }
}


int __io_putchar(int ch)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
