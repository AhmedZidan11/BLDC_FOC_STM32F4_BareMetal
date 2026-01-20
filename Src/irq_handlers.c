/**
 * @file irq_handlers.c
 * @brief Definitions for all IRQHandlers.
 *
 *  For each implemented interrupt a corresponding callback function is called
 */
#include "stm32f4xx.h"
#include "drivers/exti.h"

/* Calling EXTI-Dispatchers with corresponding EXTI_lines*/
void EXTI0_IRQHandler(void)      { exti_dispatch(0,0); }

void EXTI1_IRQHandler(void)      { exti_dispatch(1,1); }

void EXTI2_IRQHandler(void)      { exti_dispatch(2,2); }

void EXTI3_IRQHandler(void)      { exti_dispatch(3,3); }

void EXTI4_IRQHandler(void)      { exti_dispatch(4,4); }

void EXTI9_5_IRQHandler(void)    { exti_dispatch(5,9); }

void EXTI15_10_IRQHandler(void)  { exti_dispatch(10,15); }

