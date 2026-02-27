/**
 * @file usart2.c
 * @brief uart2 module implementation (STM32F4, CMSIS only).
 *
 *  Implements USART2 initialization, non-blocking TX/RX using ring buffers, and an ISR handler to be called from USART2_IRQHandler.
 *
 */

#include <drivers/usart2.h>
#include <string.h>


/**
 * @brief Computation of BRR value based on PCLK frequency and Baud rate.
 *
 * @param usart_cfg Pointer to the USART configuration instance.
 * @return USART_BRR value.
 *
 * @note CR1_OVER8 assumed 0.
 */
uint32_t baud_rate_config(const usart2_cfg_t *usart_cfg)
{
	if((usart_cfg == NULL)||(usart_cfg->usart_baud== 0u)) return 0u;

	// calculate mantissa and fraction
	uint32_t mantissa = usart_cfg->usart_pclk_hz/(16u*usart_cfg->usart_baud);

	uint32_t rem = usart_cfg->usart_pclk_hz%(16*usart_cfg->usart_baud);
	uint32_t fraction = (rem + (usart_cfg->usart_baud>>1))/usart_cfg->usart_baud;
	if (fraction >= 16u)
		{
	        fraction = 0u;
	        mantissa += 1u;
		}
	// USART_BRR = [mantissa (12bits) | fraction(4 bits)]
	return (fraction&0x0FU)|(mantissa<<4);
}

/* Configure USART registers */
/* GPIO must be configured in advance (Mode: AF, Speed: High Speed)  */
bool usart2_init(const usart2_cfg_t *usart_cfg, usart2_handle_t *usart_h)
{
	if (usart_cfg == NULL) return false;															// invalid cfg pointer
	if (usart_cfg->pin_cfg_rx->mode != GPIO_MODE_AF) return false;									// Requires GPIO mode = AF
	if (usart_cfg->pin_cfg_tx->mode != GPIO_MODE_AF) return false;									// Requires GPIO mode = AF
	if((usart_h == NULL)||(usart_h->rx_buffer==NULL)||(usart_h->tx_buffer==NULL)) return false;		// invalid handle pointers
	if(!gpio_init_pin(usart_cfg->pin_cfg_rx)) return false;											// gpio_init failed
	if(!gpio_init_pin(usart_cfg->pin_cfg_tx)) return false;

	/* reset values in USART handle */
	memset(usart_h->rx_buffer,0,sizeof(ring_buffer_t));
	memset(usart_h->tx_buffer,0,sizeof(ring_buffer_t));
	// reset overflow counters
	usart_h->rx_buffer->drop_cnt = 0;
	usart_h->tx_buffer->drop_cnt = 0;
	// reset head and tail indices
	usart_h->rx_buffer->head = 0;
	usart_h->rx_buffer->tail = 0;
	usart_h->tx_buffer->head = 0;
	usart_h->tx_buffer->tail = 0;
	// reset flags
	usart_h->err_ore_cnt = 0;
	usart_h->err_fe_cnt = 0;
	usart_h->err_ne_cnt = 0;
	usart_h->err_pe_cnt = 0;


	/* Disable USART */
	USART2->CR1 &= ~USART_CR1_UE;

	/* Enable Clock for USART2 */
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

	/* Configure Baud rate register */
	USART2->BRR = baud_rate_config(usart_cfg);

	/* Configure control register CR1. Keep default values for CR2 and CR3  */
	// Enable TE, RE and RXNEIE. TXEIE should not be enabled here
	USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
	// Disable TXEIE
	USART2->CR1 &= ~USART_CR1_TXEIE;

	/* NVIC Configuration */
	// Set priority and clear any Pending USART-interrupts
	NVIC_SetPriority(usart_cfg->irqn, usart_cfg->irq_priority);
	NVIC_ClearPendingIRQ(usart_cfg->irqn);
	NVIC_EnableIRQ(usart_cfg->irqn);

	/* USART enable */
	USART2->CR1 |= USART_CR1_UE;
	return true;
}

size_t usart2_write(usart2_handle_t *usart_h, const uint8_t* data, const size_t len)
{
	// check pointers
	if((usart_h == NULL)||(usart_h->tx_buffer == NULL)||(data == NULL)) return 0;
	// check length
	if(len == 0) return 0;

	ring_buffer_t *rb = usart_h->tx_buffer;
	size_t write_cnt;
	for(write_cnt = 0; write_cnt < len; write_cnt++)
	{
		uint16_t head = rb->head;
		uint16_t tail = rb->tail;
		// BUFFER_MASK must be 2^n, otherwise this wont work
		uint16_t next_head = (head + 1u) & BUFFER_MASK;

		if (next_head == tail)
		{
			// buffer is full, drop and increase drop counter
			rb->drop_cnt += (uint16_t)(len - write_cnt);
			break;
		}

		rb->buffer_array[head] = data[write_cnt];
		rb->head = next_head;
	}

	if(write_cnt)
	{
		USART2->CR1 |= USART_CR1_TXEIE;
	}
	return write_cnt;

}

size_t usart2_read(usart2_handle_t *usart_h, uint8_t* output, const size_t max_len)
{
	// check pointers
	if((usart_h == NULL)||(usart_h->rx_buffer == NULL)||(output == NULL)) return 0;
	// check length
	if(max_len == 0) return 0;

	ring_buffer_t *rb = usart_h->rx_buffer;
	size_t read_cnt;
	for(read_cnt = 0; read_cnt < max_len; read_cnt++)
		{
			uint16_t head = rb->head;
			uint16_t tail = rb->tail;

			if (tail == head)
			{
				// Buffer is empty
				break;
			}

			output[read_cnt] = rb->buffer_array[tail];
			// BUFFER_MASK must be 2^n, otherwise this wont work
			rb->tail = (tail + 1u) & BUFFER_MASK;
		}
		return read_cnt;
}

void usart2_irq_handler(usart2_handle_t* usart_h)
{
	// check pointers
	if ((usart_h == NULL) || (usart_h->rx_buffer == NULL) || (usart_h->tx_buffer == NULL))return;

	// get USART2 status
	uint16_t sr = (uint16_t)USART2->SR;
	uint16_t cr1 = (uint16_t)USART2->CR1;
	bool error_flag = false;

	// Check for errors and update error counters
	if(sr & USART_SR_ORE)
	{
		usart_h->err_ore_cnt++;
		usart_h->rx_buffer->drop_cnt++;
		error_flag = true;
	}
	if(sr & USART_SR_FE)
	{
		usart_h->err_fe_cnt++;
		usart_h->rx_buffer->drop_cnt++;
		error_flag = true;
	}
	if(sr & USART_SR_NE)
	{
		usart_h->err_ne_cnt++;
		usart_h->rx_buffer->drop_cnt++;
		error_flag = true;
	}
	if(sr & USART_SR_PE)
	{
		usart_h->err_pe_cnt++;
		usart_h->rx_buffer->drop_cnt++;
		error_flag = true;
	}
	if(error_flag)
	{
		// clear error after reading DR
		(void)USART2->DR;
	}
	// Copy from DR to ring buffer if RXNE interrupt is triggered
	else if(sr & USART_SR_RXNE)
	{
		uint8_t dr = (uint8_t)USART2->DR;
		// get head and tail values from rx_buffer
		uint16_t head = usart_h->rx_buffer->head;
		uint16_t tail = usart_h->rx_buffer->tail;

		uint16_t next_head = (uint16_t)((head + 1u) & BUFFER_MASK);
		if(next_head == tail)
		{
			usart_h->rx_buffer->drop_cnt++;
		}
		else
		{
			usart_h->rx_buffer->buffer_array[head] = dr;
			usart_h->rx_buffer->head = next_head;
		}

	}
	// Copy from ring buffer to DR if TXE interrupt is triggered
	if((sr & USART_SR_TXE)&&(cr1 & USART_CR1_TXEIE))
	{
		// get head and tail values from tx_buffer
		uint16_t head = usart_h->tx_buffer->head;
		uint16_t tail = usart_h->tx_buffer->tail;

		if(tail != head)
		{
			uint8_t out = usart_h->tx_buffer->buffer_array[tail];
			usart_h->tx_buffer->tail = (uint16_t)((tail + 1u) & BUFFER_MASK);
			USART2->DR = out;
		}
		else
		{
			USART2->CR1 &=  ~USART_CR1_TXEIE;
		}
	}


}
