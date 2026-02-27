#ifndef DRIVERS_USART2_H_
#define DRIVERS_USART2_H_

/**
 * @file usart2.h
 * @brief Minimal uart2 driver for STM32F4 (CMSIS only).
 *
 * Provides registers configuration and data transfer (read/write).
 * Main goal is to establish communication with PC via ST-LINK VCP.
 *
 * Responsibilities:
 * - Configure uart2 registers.
 * - Receive and transmit data.
 * - utilize ring buffer to store data.
 * - implement interrupts (RXNE & TXE).
 *
 * @note GPIO pins must be configured via GPIO driver.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "stm32f4xx.h"
#include "drivers/gpio.h"

// Configuration Constants
#define UART_BUFFER_SIZE  256U	     // Must be a power of 2
#define BUFFER_MASK       (UART_BUFFER_SIZE - 1)

/**
 * @brief Stores incoming/outgoing bytes to decouple ISR from application and reduce data loss.
 *
 */
typedef struct {
    uint8_t buffer_array[UART_BUFFER_SIZE];
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t drop_cnt;
} ring_buffer_t;

/**
 * @brief USART2 configuration.
 *
 * @pre pin_cfg_rx and pin_cfg_tx must describe correct AF mapping. Driver will call gpio_init_pin.
 *
 */
typedef struct {
	uint32_t usart_pclk_hz;
	uint32_t usart_baud;
	IRQn_Type irqn;
	uint32_t irq_priority;
	const gpio_pin_cfg_t *pin_cfg_rx;
	const gpio_pin_cfg_t *pin_cfg_tx;
}usart2_cfg_t;

/**
 * @brief Handle for USART2.
 *
 */
typedef struct {

	ring_buffer_t* rx_buffer;
	ring_buffer_t* tx_buffer;
	// USART error counters
    volatile uint32_t err_ore_cnt;
    volatile uint32_t err_fe_cnt;
    volatile uint32_t err_ne_cnt;
    volatile uint32_t err_pe_cnt;
}usart2_handle_t;

/**
 * @brief Configure USART2 registers according to the given instance.
 *
 * @param usart_cfg Pointer to the USART configuration instance.
 * @param usart_h Pointer to the USART handle instance.
 * @return true if applied, false if parameters invalid.
 */
bool  usart2_init(const usart2_cfg_t *usart_cfg, usart2_handle_t *usart_h);

/**
 * @brief Write a chunk of data in the ring buffer and set TXEIE flag.
 * If buffer is full, bytes are dropped and drop_cnt increments.
 *
 * @param usart_h Pointer to the USART handle instance.
 * @param data Pointer to the data block to be copied in the ring buffer.
 * @param len Number of Bytes to be copied in the ring buffer.
 * @return Number of successful written Bytes.
 */
size_t usart2_write(usart2_handle_t *usart_h, const uint8_t* data, const size_t len);

/**
 * @brief Read a chunk of data from the ring buffer.
 * Returns immediately with how many bytes were actually processed (0 if buffer is empty).
 *
 * @param usart_h Pointer to the USART handle instance.
 * @param output Pointer to the output data block.
 * @param max_len Maximum number of Bytes to be read.
 * @return Number of successful read Bytes.
 */
size_t usart2_read(usart2_handle_t *usart_h, uint8_t* output, const size_t max_len);

/**
 * @brief callback function, must be called from USART2_IRQHand.
 *
 * @param usart_h Pointer to the USART handle instance.
 *
 */
void usart2_irq_handler(usart2_handle_t* usart_h);




#endif /* DRIVERS_USART2_H_ */
