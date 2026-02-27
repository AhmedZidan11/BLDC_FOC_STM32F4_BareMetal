/**
 * @file irq_handlers.c
 * @brief Central IRQHandler definitions (vector table targets).
 *
 * Each ISR forwards the event to its driver-level handler/dispatcher.
 */

#include "stm32f4xx.h"
#include "drivers/exti.h"
#include "drivers/adc.h"
#include "drivers/usart2.h"
#include "drivers/systick.h"

/* IRQ handlers forward EXTI lines to the common dispatcher */
void EXTI0_IRQHandler(void)      { exti_dispatch(0,0); }
void EXTI1_IRQHandler(void)      { exti_dispatch(1,1); }
void EXTI2_IRQHandler(void)      { exti_dispatch(2,2); }
void EXTI3_IRQHandler(void)      { exti_dispatch(3,3); }
void EXTI4_IRQHandler(void)      { exti_dispatch(4,4); }
void EXTI9_5_IRQHandler(void)    { exti_dispatch(5,9); }
void EXTI15_10_IRQHandler(void)  { exti_dispatch(10,15); }

extern adc_handle_t ADC1_IN0_H;

/* ADC EOC-Interrupt, update data in ADC handle */
void ADC_IRQHandler(void)
{
	adc_irq_handler(&ADC1_IN0_H);
}

extern usart2_handle_t USART2_H;

/* USART2 ISR: handles RXNE/TXE and error flags via the driver */
void USART2_IRQHandler(void)
{
	usart2_irq_handler(&USART2_H);
}

/* SYSTICK ISR: updates systick counter */
void SysTick_Handler()
{
	SYSTICK_IrqHandler();
}
