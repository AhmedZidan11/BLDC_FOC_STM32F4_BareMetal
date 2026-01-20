
#ifndef DRIVERS_EXTI_H
#define DRIVERS_EXTI_H
/**
 * @file exti.h
 * @brief Generic module for external interrupts for STM32F4 (CMSIS only).
 *
 * Provides registers configuration, callback assignments and dispatchers
 *
 * Responsibilities: init, assign callback to corresponding exti-lines
 *
 * @note isr-safe, exti_dispatch is meant to be called by exti_IRQHandlers (see irq_handlers.c)
 */

#include <stdint.h>
#include <stdbool.h>
#include "drivers/gpio.h"
#include "stm32f4xx.h"


// function pointers for interrupts callback to be utilized in the dispatcher
typedef void (*exti_callback_t)(void *callback_arg);

/**
 * @brief identifiers for the interrupt edge
 *
 */
typedef enum {
	RISING_EDGE = 0,/**< RISING_EDGE */
	FALLING_EDGE =1,/**< FALLING_EDGE */
	BOTH_EDGES = 2, /**< BOTH_EDGES */
}exti_edge_t;

/**
 * @brief generic EXTI configuration
 *
 * Notes: GPIO pin configuration is performed within GPIO-Module
 */
typedef struct {
	const gpio_pin_cfg_t *gpio_cfg;
	exti_edge_t edge;
	uint8_t priority;
}exti_cfg_t;

/**
 * @brief Configure a defined EXTI according to a cfg-instant.
 *
 * @param cfg: Pointer to the EXTI configuration instant.
 * @return true if applied, false if parameters invalid.
 */
bool exti_init(const exti_cfg_t *cfg);

/**
 * @brief Assign a callback function to the corresponding element of a callback_array
 * 		  Based on EXTI-line
 *
 * @param line: Number of EXTI_line
 * @param callbk: handler of a callback function
 * @param callbk_arg: a Pointer to callback_arguments
 * @return
 */
bool exti_register(uint8_t line, exti_callback_t callbk, void *callbk_arg);

/**
 * @brief Dispatch a corresponding call function for pending external interrupt
 *
 * @param first: first line in EXTI
 * @param last: last line in EXTI
 *
 * Example: EXTI_1: first = 1, last = 1
 * 			EXTI_15_10: first = 10, last = 15
 */
void exti_dispatch(uint8_t first, uint8_t last);

#endif /* DRIVERS_EXTI_H */
