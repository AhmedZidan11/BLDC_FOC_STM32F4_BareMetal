/**
 * @file pwm_tim1.c
 * @brief PWM module implementation (STM32F4, CMSIS only).
 *
 * TIM1 configuration using direct register access (CMSIS).
 * Implements: mode: PWM1, center-aligned.
 */

#include "drivers/pwm_tim1.h"

/**
 * @brief Configure TIM1 registers for PWM1 mode.
 * PWM channels must be provided as gpio_pin_cfg_t pointers (NULL disables a channel).
 *
 * @param pwm_cfg Pointer to the TIM1 configuration instance.
 */
static void pwm_config_channels(const pwm_tim1_cfg_t* pwm_cfg)
{
	uint32_t ccer  = TIM1->CCER;
	uint32_t ccmr1 = TIM1->CCMR1;
	uint32_t ccmr2 = TIM1->CCMR2;

	/* Determine active channels */
	bool ch1_en = (pwm_cfg->pin_ch1 != NULL);
	bool ch2_en = (pwm_cfg->pin_ch2 != NULL);
	bool ch3_en = (pwm_cfg->pin_ch3 != NULL);

	/* Disable outputs before configuration: CCxE = 0 */
	ccer &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E);

	/* Configure CCMRx */

	// CH1: OC1M (bits 6:4), OC1PE (bit 3)
	ccmr1 &= ~(TIM_CCMR1_OC1M | TIM_CCMR1_OC1PE);
	if(ch1_en)
	{
		// Mode: PWM1
		ccmr1 |= (TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2);
		ccmr1 |= TIM_CCMR1_OC1PE;
	}

	// CH2: OC2M (bits 14:12), OC2PE (bit 11)
	ccmr1 &= ~(TIM_CCMR1_OC2M | TIM_CCMR1_OC2PE);
	if(ch2_en)
	{
		// Mode: PWM1
		ccmr1 |= (TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_2);
		ccmr1 |= TIM_CCMR1_OC2PE;
	}

	// CH3: OC3M (bits 6:4), OC3PE (bit 3) in CCMR2
	ccmr2 &= ~(TIM_CCMR2_OC3M | TIM_CCMR2_OC3PE);
	if(ch3_en)
	{
		// Mode: PWM1
		ccmr2 |= (TIM_CCMR2_OC3M_1|TIM_CCMR2_OC3M_2);
		ccmr2 |= TIM_CCMR2_OC3PE;
	}

	/*  Polarity active-high (CCxP=0) and enable outputs (CCxE=1) */
	// Clear polarity bits
	ccer &= ~(TIM_CCER_CC1P | TIM_CCER_CC2P | TIM_CCER_CC3P);
	// enable outputs for active channels
	if (ch1_en) ccer |= TIM_CCER_CC1E;
	if (ch2_en) ccer |= TIM_CCER_CC2E;
	if (ch3_en) ccer |= TIM_CCER_CC3E;

	// Write back registers
	TIM1->CCMR1 = ccmr1;
	TIM1->CCMR2 = ccmr2;
	TIM1->CCER  = ccer;
}

bool pwm_tim1_init(const pwm_tim1_cfg_t* pwm_cfg, pwm_tim1_handle_t* pwm_h)
{
	/* check pointers and GPIO configurations */
	if ((pwm_cfg == NULL)||(pwm_h == NULL)) return false;

	// At least one channel must be configured
	if (!pwm_cfg->pin_ch1 && !pwm_cfg->pin_ch2 && !pwm_cfg->pin_ch3) return false;

	// Check gpio_init
	if(pwm_cfg->pin_ch1)
	{
		if(!gpio_init_pin(pwm_cfg->pin_ch1)) return false;
	}
	if(pwm_cfg->pin_ch2)
	{
		if(!gpio_init_pin(pwm_cfg->pin_ch2)) return false;
	}
	if(pwm_cfg->pin_ch3)
	{
		if(!gpio_init_pin(pwm_cfg->pin_ch3)) return false;
	}

	// check clock and PWM frequencies
	if ((pwm_cfg->tim_clk_hz == 0)||(pwm_cfg->pwm_hz == 0)) return false;
	// center-aligned only (1..3) at this stage
	// TODO: Add configuration for Edge-aligned mode
	if (pwm_cfg->align < 1u || pwm_cfg->align > 3u) return false;

	/* Compute PSC and ARR */

	// For center-aligned, counter goes up then down, so:
	//  pwm_hz = tim_clk_hz / ((PSC+1) * 2 * (ARR+1))

	// Compute half_ticks early to validate feasibility
	uint32_t half_ticks = pwm_cfg->tim_clk_hz / (2u * pwm_cfg->pwm_hz);
	if (half_ticks < 2u) return false;

	uint16_t psc = 0u;
	while((half_ticks / (uint32_t)(psc +1u)) > 65535u)
	{
		if (psc == 0xFFFFu) return false;
		psc++;
	}
	uint32_t arr = (half_ticks / (uint32_t)(psc +1u)) - 1u;
	if (arr > 65535u) return false;

	pwm_h->arr = (uint16_t) arr;			// store for duty computation later

	/* Enable clock for TIM1 */
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

	/* Disable TIM1  */
	TIM1->CR1 &= ~TIM_CR1_CEN;

	/* Configure center-aligned mode selection */
	TIM1->CR1 &= ~TIM_CR1_CMS;
	TIM1->CR1 |= ((uint32_t)(pwm_cfg->align & 0x3u) << TIM_CR1_CMS_Pos);

	/* Set direction to up-counting	*/
	TIM1->CR1 &= ~TIM_CR1_DIR;

	/* Configure PSC and ARR registers */
	TIM1->PSC = psc;
	TIM1->ARR = (uint16_t) arr;
	TIM1->CR1 |= TIM_CR1_ARPE;

	// PWM config for CH1/CH2/CH3 (PWM mode 1 + preload) + enable outputs
	// (Implement with clear-then-set patterns for CCMR1/CCMR2 and CCER)
	pwm_config_channels(pwm_cfg);   // Configure CCMR/CCER for enabled channels

	// advanced timer output enable: MOE = 1
	TIM1->BDTR |= TIM_BDTR_MOE;

	// safe initial duty
	TIM1->CCR1 = 0u;
	TIM1->CCR2 = 0u;
	TIM1->CCR3 = 0u;

	// update event to load preloads
	TIM1->EGR = TIM_EGR_UG;

	return true;

}

bool pwm_tim1_set_duty(pwm_tim1_handle_t* pwm_h, uint8_t ch, uint16_t duty)
{
    if (pwm_h == NULL) return false;

    // saturate to [0 - ARR]
    if (duty > pwm_h->arr) duty = pwm_h->arr;

    switch (ch) {
        case 1: TIM1->CCR1 = duty; break;
        case 2: TIM1->CCR2 = duty; break;
        case 3: TIM1->CCR3 = duty; break;
        default: return false;
    }
    return true;
}


bool pwm_tim1_start(pwm_tim1_handle_t* pwm_h)
{
    if (!pwm_h) return false;

    // Enable output gate
    TIM1->BDTR |= TIM_BDTR_MOE;

    // Start counter
    TIM1->CR1 |= TIM_CR1_CEN;

    return true;
}

bool pwm_tim1_stop(pwm_tim1_handle_t* pwm_h)
{
    if (!pwm_h) return false;

    // Stop counter
    TIM1->CR1 &= ~TIM_CR1_CEN;

    // Disable output gate
    TIM1->BDTR &= ~TIM_BDTR_MOE;

    return true;
}
