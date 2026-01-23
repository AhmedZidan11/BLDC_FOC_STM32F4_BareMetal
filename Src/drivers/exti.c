/**
 * @file exti.c
 * @brief EXTI module implementation (STM32F4, CMSIS only).
 *
 *  EXTI line configuration + callback registration + dispatcher (CMSIS)
 *  Maps GPIO pin to EXTI line via SYSCFG, configures trigger and NVIC
 *
 */

#include "drivers/exti.h"

// Arrays of 16 EXTI callback functions and their arguments
static exti_callback_t callbk_array[16] = {0};
static void *callbk_arg_array[16] = {0};

/* Configure EXTI-registers */
bool exti_init(const exti_cfg_t *cfg)
{
	if (cfg == NULL) return false;								// invalid cfg pointer
	if (cfg->gpio_cfg->mode != GPIO_MODE_INPUT) return false;	// Requires GPIO input mode (EXTI is edge/level on input pin)
	if(!gpio_init_pin(cfg->gpio_cfg)) return false;				// gpio_init failed


	uint8_t line = cfg->gpio_cfg->pin.pin;	// EXTI_line for GPIO pin
	uint8_t line_idx = line/4;				// index for SYSCFG_EXTICR
	uint8_t line_shift = (line%4)*4;		// Bit-shift for SYSCFG_EXTICR
	uint8_t exticr_value;					// Value to be written in SYSCFG_EXTICR

	switch (cfg->gpio_cfg->pin.port_name)
	{
	case GPIO_PORTA: exticr_value=0; break;
	case GPIO_PORTB: exticr_value=1; break;
	case GPIO_PORTC: exticr_value=2; break;
	case GPIO_PORTD: exticr_value=3; break;
	case GPIO_PORTE: exticr_value=4; break;
	case GPIO_PORTF: exticr_value=5; break;
	case GPIO_PORTG: exticr_value=6; break;
	case GPIO_PORTH: exticr_value=7; break;
	default: return false;
	}

	// Enable SYSCFG clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// configure the corresponding interrupt
	SYSCFG->EXTICR[line_idx] &= ~(0xFu << line_shift);
	SYSCFG->EXTICR[line_idx] |= ((uint32_t)exticr_value<<line_shift);

	// Mask interrupt
	EXTI->IMR &= ~(1u<<line);

	// set interrupt trigger edge
	if(cfg->edge == RISING_EDGE)
	{
		EXTI->FTSR &= ~(1u << line);
		EXTI->RTSR |= (1u<<line);
	}
	else if(cfg->edge == FALLING_EDGE)
	{
		EXTI->RTSR &= ~(1u << line);
		EXTI->FTSR |= (1u<<line);
	}
	else if(cfg->edge == BOTH_EDGES)
	{
		EXTI->RTSR &= ~(1u << line);
		EXTI->FTSR &= ~(1u << line);
		EXTI->RTSR |= (1u<<line);
		EXTI->FTSR |= (1u<<line);
	}
	else { return false; }

	// clear pending
	EXTI->PR = (1u<<line);

	// unmask interrupt
	EXTI->IMR |= (1u<<line);

	// set priority and enable interrupt
	switch(line)
	{
	case 0: NVIC_SetPriority(EXTI0_IRQn,cfg->priority);
			NVIC_ClearPendingIRQ(EXTI0_IRQn);
			NVIC_EnableIRQ(EXTI0_IRQn);
			break;
	case 1: NVIC_SetPriority(EXTI1_IRQn,cfg->priority);
			NVIC_ClearPendingIRQ(EXTI1_IRQn);
			NVIC_EnableIRQ(EXTI1_IRQn);
			break;
	case 2: NVIC_SetPriority(EXTI2_IRQn,cfg->priority);
			NVIC_ClearPendingIRQ(EXTI2_IRQn);
			NVIC_EnableIRQ(EXTI2_IRQn);
			break;
	case 3: NVIC_SetPriority(EXTI3_IRQn,cfg->priority);
			NVIC_ClearPendingIRQ(EXTI3_IRQn);
			NVIC_EnableIRQ(EXTI3_IRQn);
			break;
	case 4: NVIC_SetPriority(EXTI4_IRQn,cfg->priority);
			NVIC_ClearPendingIRQ(EXTI4_IRQn);
			NVIC_EnableIRQ(EXTI4_IRQn);
			break;
	case 5: case 6: case 7: case 8: case 9: NVIC_SetPriority(EXTI9_5_IRQn,cfg->priority);
											NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
											NVIC_EnableIRQ(EXTI9_5_IRQn);
											break;
	case 10: case 11: case 12: case 13: case 14: case 15: NVIC_SetPriority(EXTI15_10_IRQn,cfg->priority);
														  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
														  NVIC_EnableIRQ(EXTI15_10_IRQn);
														  break;
	default: return false;
	}
	return true;
}

/* Assign callback function and callback arguments to the corresponding array elements */
bool exti_register(uint8_t line, exti_callback_t callbk, void *callbk_arg)
{
	if (line > 15u) return false;	// invalid line number
	callbk_array[line] = callbk;
	callbk_arg_array[line] = callbk_arg;
	return true;
}

/* Determine which EXTI is pending and call the corresponding callback function */
void exti_dispatch(uint8_t first, uint8_t last)
{
    uint32_t pending = EXTI->PR;
    for(uint8_t line = first; line <= last; line++)
    {
    	if (pending & (1u << line))
    	{
    		EXTI->PR = (1u << line);    						// clear pending
    		// if a callback is registered for the pending EXTI_line, call it
    		if (callbk_array[line]) callbk_array[line](callbk_arg_array[line]);
    	}
    }
}

