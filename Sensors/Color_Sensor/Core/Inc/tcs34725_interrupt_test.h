/**
 * @file tcs34725_interrupt_test.h
 * @brief TCS34725 color sensor interrupt mode test declarations
 *
 * Include this header in main.c to access the two test functions.
 */

#ifndef TCS34725_INTERRUPT_TEST_H
#define TCS34725_INTERRUPT_TEST_H

#include "main.h"
#include "driver_tcs34725_interrupt.h"

/* ------------------------------------------------------------------ */
/* Test configuration — edit these to match your setup                  */
/* ------------------------------------------------------------------ */

/**
 * Number of RGBC samples to collect before the test stops.
 */
#define TEST_READ_COUNT          20U

/**
 * Clear channel value below which the sensor fires an interrupt.
 * Lower this in bright environments.
 */
#define INTERRUPT_LOW_THRESHOLD  100U

/**
 * Clear channel value above which the sensor fires an interrupt.
 * Raise this in bright environments.
 */
#define INTERRUPT_HIGH_THRESHOLD 1000U

/**
 * Interrupt persistence mode.
 * TCS34725_INTERRUPT_MODE_EVERY_RGBC_CYCLE fires on every out-of-range
 * cycle, which is the most responsive setting for testing.
 */
#define INTERRUPT_MODE  TCS34725_INTERRUPT_MODE_EVERY_RGBC_CYCLE

/**
 * Delay in milliseconds between consecutive reads.
 * Must be greater than the sensor integration time (24 ms default).
 */
#define READ_DELAY_MS            50U

/* ------------------------------------------------------------------ */
/* Shared interrupt flag                                                */
/* Set to 1 inside HAL_GPIO_EXTI_Callback when PA0 fires.              */
/* Cleared inside tcs34725_interrupt_test after each read.             */
/* ------------------------------------------------------------------ */
extern volatile uint8_t g_interrupt_flag;

/* ------------------------------------------------------------------ */
/* Public function declarations                                         */
/* ------------------------------------------------------------------ */

/**
 * @brief Interrupt-driven test.
 *        Waits for the TCS34725 INT pin to fire, reads RGBC data,
 *        and prints results over UART2. Stops after TEST_READ_COUNT
 *        samples.
 */
void tcs34725_interrupt_test(void);

/**
 * @brief Polling fallback test.
 *        Reads 10 RGBC samples every READ_DELAY_MS without using the
 *        INT pin. Use this first to confirm I2C communication works.
 */
void tcs34725_polling_test(void);

#endif /* TCS34725_INTERRUPT_TEST_H */
