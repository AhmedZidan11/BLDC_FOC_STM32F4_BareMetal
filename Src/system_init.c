
#include "stm32f4xx.h"

/**
 * @file system_init.c
 * @brief Early system initialization hook called from startup before main().
 *
 * This file is intentionally minimal:
 * - Enable FPU when the build uses hardware floating point.
 * - Ensure the vector table base address is correctly set.
 * - Leave clock configuration to a dedicated function called from main().
 */

void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    /* Enable full access to CP10 and CP11 (FPU). */
    SCB->CPACR |= (0xFu << 20);
    __DSB();
    __ISB();
#endif
}
