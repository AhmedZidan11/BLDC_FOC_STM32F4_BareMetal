#ifndef DRIVERS_SYSTICK_H_
#define DRIVERS_SYSTICK_H_

/**
 * @file systick.h
 * @brief Minimal SysTick driver for STM32F4 (CMSIS only).
 *
 * Provides registers configuration and set-up a system clock to track and schedule tasks.
 * Designed for use-cases with a single SysTick instance with a lifetime equals the program lifetime.
 *
 * Responsibilities:
 * - Configure SysTick registers.
 * - Compute Elapsed time in ms and us (Approximately).
 * - implement SysTick interrupts.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32f4xx.h"
/**
 * @brief SysTick configuration.
 *
 */
typedef struct {
  uint32_t sysclk_hz;   	 // Core clock used by SysTick (CPU clock source)
  uint32_t tick_period_us;   // SysTick interrupt period in microseconds
  uint32_t irq_prio;         // SysTick IRQ priority
} systick_cfg_t;

/**
 * @brief SysTick Handler
 *
 */
typedef struct {
  volatile uint32_t tick_count;   // Increments by SysTick IRQ handler
} systick_handle_t;

bool SYSTICK_Init(const systick_cfg_t *systick_cfg, systick_handle_t* systick_h);

void SYSTICK_IrqHandler(void);

uint32_t SYSTICK_GetTick(void);
uint32_t SYSTICK_GetTimeMs(void);
uint64_t SYSTICK_GetTimeUs(void);

/* Wrap-around safe elapsed helpers */
static inline uint32_t SYSTICK_ElapsedMs(uint32_t now_ms, uint32_t start_ms)
{
  return (uint32_t)(now_ms - start_ms);
}

static inline uint64_t SYSTICK_ElapsedUs(uint64_t now_us, uint64_t start_us)
{
  return (uint64_t)(now_us - start_us);
}

#endif /* DRIVERS_SYSTICK_H_ */


