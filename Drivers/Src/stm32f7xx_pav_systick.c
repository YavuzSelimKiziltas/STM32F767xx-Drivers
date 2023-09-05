/*
 * stm32f7xx_pav_systick.c
 *
 *  Created on: Aug 29, 2023
 *      Author: Yavuz
 */

#include "stm32f7xx_pav_systick.h"

volatile uint32_t uwTick;
#define uwTickStep		1

/* PM0253 pg. 216 */
void Systick_Config()
{
	SYSCFG_CLK_EN();

	RCC->APB1ENR |= (1 << 28);	// Enable PWR Clock

	SYSTICK->RVR = (SYS_FREQUENCY) / (1000 / uwTickStep);

	SYSTICK->CVR = 0;

	SYSTICK->CSR = (1 << SYSTICK_CSR_EN)      |
				   (1 << SYSTICK_CSR_TICKINT) |
				   (1 << SYSTICK_CSR_CLOCK)   ;
}

uint32_t PAV_GetTick()
{
	return uwTick;
}

void PAV_IncTick()
{
	uwTick += uwTickStep;
}

void PAV_Delay(uint32_t delay)
{
	uint32_t tickStart;
	uint32_t wait = delay;

	tickStart = PAV_GetTick();

	if (wait < 0xFFFFFF)
	{
	    wait += (uint32_t)(uwTickStep);
	}

	while((PAV_GetTick() - tickStart) < wait)
	{
	}

}

void SysTick_Handler()
{
	PAV_IncTick();
}
