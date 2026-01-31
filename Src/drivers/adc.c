/**
 * @file adc.c
 * @brief ADC driver implementation (STM32F4, CMSIS only).
 *
 *  Notes:
 *  - Handles EOC interrupt only (no overrun handling).
 */


#include "drivers/adc.h"

/* Configure ADC registers */
bool adc_init(adc_handle_t *adc_h, const adc_cfg_t *adc_cfg)
{
	// Validate input pointers
	if(adc_h == NULL || adc_cfg == NULL) return false; // invalid handle and cfg instances

	if(adc_h->cfg == NULL || adc_h->inst == NULL) return false; // invalid instances in adc_h

	if(adc_cfg->inst == NULL) return false; // invalid instance in adc_cfg

	if(adc_cfg->adc_channel > ADC_CHANNEL_MAX) return false; // invalid channel number

	if(adc_cfg->pin_cfg->mode != GPIO_MODE_ANALOG) return false; // invalid pin mode

	// Configure GPIO Pin
	if(!gpio_init_pin(adc_cfg->pin_cfg)) return false;

	// Enable clock source for ADC
	if(adc_cfg->inst == ADC1){
		RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	}
	else if(adc_cfg->inst == ADC2){
		RCC->APB2ENR |= RCC_APB2ENR_ADC2EN;
	}
	else if(adc_cfg->inst == ADC3){
		RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
	}

	// Disable ADC before configuration
	adc_cfg->inst->CR2 &= ~ ADC_CR2_ADON;

	// set conversion resolution
	adc_cfg->inst->CR1 &= ~(ADC_CR1_RES_0|ADC_CR1_RES_1);
	adc_cfg->inst->CR1 |=(adc_cfg->resolution<<ADC_CR1_RES_Pos);

	// set conversion mode
	adc_cfg->inst->CR2 &= ~ADC_CR2_CONT;
	adc_cfg->inst->CR2 |= (adc_cfg->mode<<ADC_CR2_CONT_Pos);

	// Enable End-of-conversion interrupt
	adc_cfg->inst->CR1 |= ADC_CR1_EOCIE;

	/* Single-channel regular sequence only (L = 0) */
	// set conversion sequence start
	adc_cfg->inst->SQR1 &= ~(ADC_SQR1_L_0|ADC_SQR1_L_1|ADC_SQR1_L_2|ADC_SQR1_L_3);

	// set channel number for the 1st and only conversion here
	adc_cfg->inst->SQR3 = 0;					// clear register
	adc_cfg->inst->SQR3 = adc_cfg->adc_channel; // set channel

	// configure sampling time
	uint32_t adc_sample_shift;
	if(adc_cfg->adc_channel < 10){
		adc_sample_shift = (uint32_t) adc_cfg->adc_channel*3U;
		adc_cfg->inst->SMPR2 &= ~(7U << adc_sample_shift);
		adc_cfg->inst->SMPR2 |= ((uint32_t) adc_cfg->sample_time << adc_sample_shift);
	}
	else if(adc_cfg->adc_channel >= 10){
		adc_sample_shift = (uint32_t)(adc_cfg->adc_channel - 10U)*3U;
		adc_cfg->inst->SMPR1 &= ~(7U << adc_sample_shift);
		adc_cfg->inst->SMPR1 |= ((uint32_t)adc_cfg->sample_time << adc_sample_shift);
	}

	// Clear any stale flags by reading SR and DR registers
	(void) adc_cfg->inst->SR;
	(void) adc_cfg->inst->DR;

	// Set interrupt priority for NVIC
	NVIC_SetPriority(adc_cfg->irqn, adc_cfg->irq_priority);
	NVIC_ClearPendingIRQ(adc_cfg->irqn);
	NVIC_EnableIRQ(adc_cfg->irqn);

	// initialize handle states
	adc_h->inst = adc_cfg->inst;
	adc_h->last_reading = 0;
	adc_h->adc_data_ready = false;


	// Enable ADC
	adc_cfg->inst->CR2 |= ADC_CR2_ADON;


	return true;

}


/* Starting conversion, call once in continuous mode. Call before each conversion in single mode */
void adc_start(adc_handle_t *adc_h)
{
	adc_h->adc_data_ready = false;
	adc_h->inst->CR2 |= ADC_CR2_SWSTART;
}


/* read new data when available */
bool adc_read(adc_handle_t *adc_h, uint16_t *output)
{
	if((adc_h == NULL)||(output==NULL)){
		return false;
	}
	if((adc_h->inst == NULL)||(adc_h->cfg==NULL)){
		return false;
	}
	if(adc_h->adc_data_ready){
		*output = adc_h->last_reading;
		adc_h->adc_data_ready = false;
		return true;
	}
	return false;
}


/* callback function for EOC, overrun cases are not considered */
void adc_irq_handler(adc_handle_t *adc_h)
{
	if((adc_h == NULL)||(adc_h->inst == NULL)||(adc_h->cfg==NULL)) return; // Do nothing if pointers invalid

	if(adc_h->inst->SR & ADC_SR_EOC){
		adc_h->last_reading = (uint16_t)(adc_h->inst->DR & 0xFFFFU);
		adc_h->adc_data_ready = true;
	}

}
