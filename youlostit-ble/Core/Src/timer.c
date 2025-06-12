/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"
#include "leds.h"

// Define global variables to track the LED state
volatile uint8_t led_state = 0;


void timer_init(TIM_TypeDef* timer)
{
  // TODO implement this

	// 1. Enable the clock for TIM2 in the RCC register
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

	// 2. Stop the timer (disable the counter)
	timer->CR1 &= ~TIM_CR1_CEN;

	// 3. Reset all counters and states
	timer->CNT = 0;       // Reset the counter
	timer->EGR = TIM_EGR_UG;  // Force an update event to reset internal flags
	timer->SR = 0;        // Clear all status flags

	// 4. Configure the timer for auto-reload
	timer->CR1 |= TIM_CR1_ARPE;  // Enable Auto-Reload Preload
	timer->ARR = 0xFFFF;         // Set a default max auto-reload value (will be updated in timer_set_ms)

	// 5. Enable interrupts for update events
	timer->DIER |= TIM_DIER_UIE;  // Enable update interrupt (UIE)

	// 6. Configure the NVIC for TIM2
	NVIC_SetPriority(TIM2_IRQn, 1);   // Set priority level
	NVIC_EnableIRQ(TIM2_IRQn);       // Enable TIM2 interrupt in the NVIC

	// 7. Start the timer
	timer->CR1 |= TIM_CR1_CEN;

}

void timer_reset(TIM_TypeDef* timer)
{
    // TODO implement this
    // Reset only the counter value
    timer->CNT = 0;

}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms) {
    uint16_t prescaler = 4000;  // Prescaler value for 50ms period
    uint32_t arr_value = 50;    // ARR value for 50ms period

    // Set the prescaler (PSC)
    timer->PSC = prescaler;  // Prescaler is 0-based, so subtract 1

    // Set the auto-reload register (ARR)
    timer->ARR = arr_value;  // ARR is 0-based, so subtract 1

    // Reset the timer counter
    timer_reset(timer);

    // Force an update event to apply PSC and ARR settings
    timer->EGR = TIM_EGR_UG;
}
