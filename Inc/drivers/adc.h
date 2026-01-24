#ifndef DRIVERS_ADC_H
#define DRIVERS_ADC_H
/**
 * @file adc.h
 * @brief Minimal ADC driver for STM32F4 (CMSIS only).
 *
 * Supports single-channel regular conversions (single-shot or continuous).
 *
 *
 * Responsibilities:
 * - Configure ADC instance (resolution, sampling time, channel sequence length = 1)
 * - Start conversion
 * - Read last conversion result
 * - Handle EOC interrupt via adc_irq_handler() (called from ADC_IRQHandler)
 *
 * @note This driver configures a single regular channel only (no scan, no injected channels)
 */

#include "stm32f4xx.h"
#include "drivers/gpio.h"

#define ADC_CHANNEL_MAX 18U		/**< Max regular channel index (0..18) for STM32F446RE */

/**
 * @brief identifiers for ADC conversion modes in ADC_CR2
 *
 */
typedef enum {
	ADC_MODE_CONTINUOUS = 0U,    /**< Continuous conversion (CONT=1 */
	ADC_MODE_SINGLE = 1U,        /**< Single conversion (CONT=0) */
} adc_mode_t;

/**
 * @brief identifiers for ADC resolutions in ADC_CR1
 *
 */
typedef enum {
	ADC_12_BIT = 0x00,/**< 12 bit */
	ADC_10_BIT = 0x01,/**< 10 bit */
	ADC_8_BIT = 0x02, /**< 8 bit */
	ADC_6_BIT = 0x03, /**< 6 bit */
} adc_resolution_t;

/**
 * @brief identifiers for ADC sampling time in ADC_SMPR2
 *
 */
typedef enum {
	CYCLES_3 = 0x00,  /**< 3 cycles */
	CYCLES_15 = 0x01, /**< 15 cycles */
	CYCLES_28 = 0x02, /**< 28 cycles */
	CYCLES_56 = 0x03, /**< 56 cycles */
	CYCLES_84 = 0x04, /**< 84 cycles */
	CYCLES_112 = 0x05,/**< 112 cycles */
	CYCLES_144 = 0x06,/**< 144 cycles */
	CYCLES_480 = 0x07,/**< 480 cycles */
} adc_sample_t;


/**
 * @brief ADC configuration (single-channel, regular group).
 *
 * @pre pin_cfg must be configured as analog input (GPIO_MODE_ANALOG).
 * @note EOC interrupt is enabled by this driver.
 *
 */
typedef struct {
	ADC_TypeDef *inst; // ADC1, ADC2, ADC3
	uint8_t adc_channel;
	// Continuous or single only
	adc_mode_t mode;
	// sample time selection
	adc_sample_t sample_time;
	// Interrupt handle
	IRQn_Type irqn;
	// Interrupt priority
	uint8_t irq_priority;
	// GPIO pin as analog channel
	const gpio_pin_cfg_t *pin_cfg;
	// ADC resolution
	adc_resolution_t resolution;
} adc_cfg_t;

/**
 * @brief Handle for ADC data
 *
 *
 */
typedef struct {
	ADC_TypeDef *inst;
	volatile uint16_t last_reading;
	volatile bool adc_data_ready;
	const adc_cfg_t *cfg;

} adc_handle_t;

/**
 * @brief Configure ADC unit according to the given instance.
 *
 * @param adc_h Pointer to ADC handle
 * @param adc_cfg Pointer to ADC configuration
 * @return true if applied, false if parameters invalid.
 */
bool adc_init(adc_handle_t *adc_h, const adc_cfg_t *adc_cfg);

/**
 * @brief trigger ADC conversion
 *
 * @param adc_h Pointer to ADC handle
 */
void adc_start(adc_handle_t *adc_h);

/**
 * @brief check if new reading is available and update adc_h
 *
 * @param adc_h Pointer to ADC handle
 * @param value Pointer to variable to store ADC reading
 * @return true if applied, false if parameters invalid
 */
bool adc_read(adc_handle_t *adc_h, uint16_t *value);

/**
 * @brief callback function for EOC-interrupt called by ADC_IRQHandler
 *
 * @param adc_h Pointer to ADC handle
 */
void adc_irq_handler(adc_handle_t *adc_h);


#endif /* DRIVERS_ADC_H */
