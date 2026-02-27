/**
 * @file systick.c
 * @brief SysTick driver implementation (STM32F4, CMSIS only).
 *
 * Initializes SysTick for a configurable tick period and provides coarse (ms) and best-effort (us) timestamps.
 * SysTick_Handler() must call SYSTICK_IrqHandler().
 *
 */

#include <drivers/systick.h>

static const systick_cfg_t    *s_cfg  = NULL;
static systick_handle_t       *s_h    = NULL;
static uint32_t               s_load = 0;


/**
 * @brief Configure SysTick to generate periodic interrupts.
 *
 * Stores cfg/handle references internally (single-instance design).
 *
 * @param systick_cfg Driver configuration.
 * @param systick_h   Driver handle (tick counter storage).
 * @return true on success, false on invalid parameters or out-of-range period.
 */
bool SYSTICK_Init(const systick_cfg_t *systick_cfg, systick_handle_t* systick_h)
{
  if ((systick_cfg == NULL)||(systick_h == NULL)) return false;
  if (systick_cfg->sysclk_hz == 0u) return false;
  if (systick_cfg->tick_period_us == 0u) return false;

  s_cfg = systick_cfg;
  s_h   = systick_h;
  s_h->tick_count = 0u;

  /* Compute cycles per tick (use 64-bit to avoid overflow). */
  const uint64_t cycles_per_tick = ((uint64_t)s_cfg->sysclk_hz * (uint64_t)s_cfg->tick_period_us) / 1000000ull;

  if (cycles_per_tick == 0ull) return false;

  const uint64_t load_u64 = cycles_per_tick - 1ull;
  if (load_u64 > 0x00FFFFFFull) return false;

  s_load = (uint32_t)load_u64;

  // Disable SysTick
  SysTick->CTRL = 0u;

  // Configure SysTick registers
  SysTick->LOAD = s_load;
  SysTick->VAL  = 0u;

  // Set Interrupt priority
  NVIC_SetPriority(SysTick_IRQn, systick_cfg->irq_prio);

  // Set CLKSOURCE = CPU, TICKINT = 1, ENABLE = 1
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                  SysTick_CTRL_TICKINT_Msk   |
                  SysTick_CTRL_ENABLE_Msk;

  return true;
}

/**
 * @brief SysTick IRQ service routine (increment-only).
 *
 * Call from SysTick_Handler().
 */
void SYSTICK_IrqHandler(void)
{
  if (s_h) { s_h->tick_count++; }
}

/**
 * @brief Get current tick_counter
 *
 * @return current tick_counter or 0 if SysTick handler is invalid
 */
uint32_t SYSTICK_GetTick(void)
{
  return (s_h) ? s_h->tick_count : 0u;
}


/**
 * @brief Get elapsed time in milliseconds (quantized to the tick period).
 *
 * Resolution follows s_cfg->tick_period_us:
 * - For tick >= 1ms, time advances in multiples of that tick (e.g. 10ms steps).
 * - For tick <  1ms, time advances in ~1ms steps due to integer conversion.
 *
 * @return Milliseconds timpestamp
 */
uint32_t SYSTICK_GetTimeMs(void)
{
  if (!s_cfg || !s_h) return 0u;
  const uint64_t total_us = (uint64_t)s_h->tick_count * (uint64_t)s_cfg->tick_period_us;
  return (uint32_t)(total_us / 1000ull);
}

/**
 * @brief Get a best-effort timestamp in microseconds since initialization.
 *
 * Combines the tick counter with the current SysTick down-counter value.
 * Uses bounded retries to reduce mixing across the tick boundary.
 *
 * @return Microseconds timestamp (best-effort).
 */
uint64_t SYSTICK_GetTimeUs(void)
{
  if (!s_cfg || !s_h) return 0ull;

  /* Double-sample tick counter around VAL to avoid rollover mixing with bounded retries. */
  for (uint32_t i = 0u; i < 3u; ++i)
  {
    const uint32_t t1  = s_h->tick_count;
    const uint32_t val = SysTick->VAL;
    const uint32_t t2  = s_h->tick_count;

    if (t1 == t2)
    {
      /* Compute elapsed cycles within the current tick */
      const uint32_t elapsed_cycles = (s_load - val);
      const uint64_t us_in_tick = ((uint64_t)elapsed_cycles * 1000000ull) / (uint64_t)s_cfg->sysclk_hz;

      return ((uint64_t)t1 * (uint64_t)s_cfg->tick_period_us) + us_in_tick;
    }
  }

  /* Fallback: tick precision only (bounded execution). */
  return (uint64_t)s_h->tick_count * (uint64_t)s_cfg->tick_period_us;
}
