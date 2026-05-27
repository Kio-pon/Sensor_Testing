# NERC Complete Sensor Matrix and DMA Optimization Change log

This document outlines the complete architectural mapping and step-by-step implementation for migrating **all 3 sensor arrays** (Front, Sensor2, Sensor3) to fully parallel, non-blocking **DMA Circular Mode with Ping-Pong Double-Buffering and EMA filtering**.

---

## 1. Pin Layout & ADC Allocation

The physical sensor array is split across **ADC1**, **ADC2**, and **ADC3** to optimize parallel performance.

### Array 1: Front Sensor Array (Sensor 1 - 8 Channels)
* Driven by: **`ADC1`**
* DMA Controller: **`DMA1` Channel 1**
* Buffering: Size **16** (8 channels $\times$ 2 for Ping-Pong)
* Physical Mapping:
  | Name | ADC Input Channel | MCU GPIO Pin |
  |---|---|---|
  | Sensor1_D1 | ADC1_IN2 | PA1 |
  | Sensor1_D2 | ADC1_IN3 | PA2 |
  | Sensor1_D3 | ADC1_IN4 | PA3 |
  | Sensor1_D4 | ADC1_IN6 | PC0 |
  | Sensor1_D5 | ADC1_IN7 | PC1 |
  | Sensor1_D6 | ADC1_IN8 | PC2 |
  | Sensor1_D7 | ADC1_IN9 | PC3 |
  | Sensor1_D8 | ADC1_IN5 | PF4 |

---

### Array 2: Side Array A (Sensor 2 - 6 Channels)
* Driven by: **`ADC3`** (Partially shared with Sensor3)
* DMA Controller: **`DMA2` Channel 1** (Handles 7 channels total: 6 for Sensor2 + 1 for Sensor3_D6)
* Buffering: Size **14** (7 channels $\times$ 2 for Ping-Pong)
* Physical Mapping:
  | Name | ADC Input Channel | MCU GPIO Pin |
  |---|---|---|
  | Sensor2_D1 | ADC3_IN1 | PB1 |
  | Sensor2_D2 | ADC3_IN2 | PE9 |
  | Sensor2_D3 | ADC3_IN3 | PE13 |
  | Sensor2_D4 | ADC3_IN5 | PB13 |
  | Sensor2_D5 | ADC3_IN6 | PE8 |
  | Sensor2_D6 | ADC3_IN7 | PD10 |

---

### Array 3: Side Array B (Sensor 3 - 6 Channels)
* Driven by: **`ADC2`** (Channels D1-D5) & **`ADC3`** (Channel D6)
* DMA Controller:
  * **`DMA2` Channel 3** (Handles 5 channels on ADC2)
  * **`DMA2` Channel 1** (Handles 1 channel on ADC3: `Sensor3_D6`)
* Buffering (ADC2): Size **10** (5 channels $\times$ 2 for Ping-Pong)
* Physical Mapping:
  | Name | ADC Input Channel | MCU GPIO Pin |
  |---|---|---|
  | Sensor3_D1 | ADC2_IN1 | PA4 |
  | Sensor3_D2 | ADC2_IN2 | PA5 |
  | Sensor3_D3 | ADC2_IN3 | PA6 |
  | Sensor3_D4 | ADC2_IN4 | PA7 |
  | Sensor3_D5 | ADC2_IN5 | PC4 |
  | Sensor3_D6 | ADC3_IN8 | PD11 | (Linked with `ADC3` circular scan)

---

## 2. Buffer Structures & Allocations

Declare these buffers globally in `main.c`:

```c
#define SENSOR1_CHANNELS 8
#define SENSOR2_TOTAL    6
#define SENSOR3_ADC2_CH  5

// Ping-Pong DMA Buffers
volatile uint16_t adc1_dma_buffer[SENSOR1_CHANNELS * 2]; // 16 elements (Front QTR)
volatile uint16_t adc2_dma_buffer[SENSOR3_ADC2_CH * 2];  // 10 elements (Sensor3 D1-D5)
volatile uint16_t adc3_dma_buffer[7 * 2];                // 14 elements (Sensor2 D1-D6 + Sensor3 D6)

// Filtered Output values (read instantly in main control loop)
volatile uint16_t sensor1_filtered[SENSOR1_CHANNELS] = {0};
volatile uint16_t sensor2_filtered[SENSOR2_TOTAL] = {0};
volatile uint16_t sensor3_filtered[6] = {0}; // D1 to D6

const float EMA_ALPHA = 0.25f; // EMA noise reduction constant
```

---

## 3. Interrupt Conversion Callbacks (Ping-Pong + EMA Filter)

Inside `main.c`, when the DMA completes half or full transfers for each ADC peripheral, the CPU filters it instantly:

```c
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) 
{
    // ── ADC1: Sensor 1 (Front Array) ──
    if (hadc->Instance == ADC1) {
        for (int i = 0; i < SENSOR1_CHANNELS; i++) {
            uint16_t raw = adc1_dma_buffer[i];
            sensor1_filtered[i] = (uint16_t)((EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * sensor1_filtered[i]));
        }
    }
    
    // ── ADC2: Sensor 3 [D1-D5] (Right Side) ──
    if (hadc->Instance == ADC2) {
        for (int i = 0; i < SENSOR3_ADC2_CH; i++) {
            uint16_t raw = adc2_dma_buffer[i];
            sensor3_filtered[i] = (uint16_t)((EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * sensor3_filtered[i]));
        }
    }
    
    // ── ADC3: Sensor 2 [D1-D6] (Left Side) & Sensor 3 [D6] ──
    if (hadc->Instance == ADC3) {
        // Sensor 2 (D1 - D6)
        for (int i = 0; i < SENSOR2_TOTAL; i++) {
            uint16_t raw = adc3_dma_buffer[i];
            sensor2_filtered[i] = (uint16_t)((EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * sensor2_filtered[i]));
        }
        // Sensor 3 (D6)
        uint16_t raw_s3d6 = adc3_dma_buffer[6];
        sensor3_filtered[5] = (uint16_t)((EMA_ALPHA * raw_s3d6) + ((1.0f - EMA_ALPHA) * sensor3_filtered[5]));
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) 
{
    // ── ADC1: Sensor 1 (Front Array) ──
    if (hadc->Instance == ADC1) {
        for (int i = 0; i < SENSOR1_CHANNELS; i++) {
            uint16_t raw = adc1_dma_buffer[SENSOR1_CHANNELS + i];
            sensor1_filtered[i] = (uint16_t)((EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * sensor1_filtered[i]));
        }
    }
    
    // ── ADC2: Sensor 3 [D1-D5] (Right Side) ──
    if (hadc->Instance == ADC2) {
        for (int i = 0; i < SENSOR3_ADC2_CH; i++) {
            uint16_t raw = adc2_dma_buffer[SENSOR3_ADC2_CH + i];
            sensor3_filtered[i] = (uint16_t)((EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * sensor3_filtered[i]));
        }
    }
    
    // ── ADC3: Sensor 2 [D1-D6] (Left Side) & Sensor 3 [D6] ──
    if (hadc->Instance == ADC3) {
        // Sensor 2 (D1 - D6)
        for (int i = 0; i < SENSOR2_TOTAL; i++) {
            uint16_t raw = adc3_dma_buffer[7 + i];
            sensor2_filtered[i] = (uint16_t)((EMA_ALPHA * raw) + ((1.0f - EMA_ALPHA) * sensor2_filtered[i]));
        }
        // Sensor 3 (D6)
        uint16_t raw_s3d6 = adc3_dma_buffer[7 + 6];
        sensor3_filtered[5] = (uint16_t)((EMA_ALPHA * raw_s3d6) + ((1.0f - EMA_ALPHA) * sensor3_filtered[5]));
    }
}
```

---

## 4. Initialization and Startup Sequence

Right before the main loop inside `main.c`, initialize DMA transfers for all three converters:

```c
// Enable circular DMA for all ADCs
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_dma_buffer, SENSOR1_CHANNELS * 2);
HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_dma_buffer, SENSOR3_ADC2_CH * 2);
HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc3_dma_buffer, 7 * 2);
```

All 20 sensor elements are now completely updated in the background without a single microsecond of CPU waiting!
