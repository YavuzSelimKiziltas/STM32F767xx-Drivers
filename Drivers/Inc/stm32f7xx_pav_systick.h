/*
 * stm32f7xx_pav_systick.h
 *
 *  Created on: Aug 29, 2023
 *      Author: Yavuz
 */

#ifndef DRIVERS_INC_STM32F7XX_PAV_SYSTICK_H_
#define DRIVERS_INC_STM32F7XX_PAV_SYSTICK_H_

#include "stm32f7xx_pav.h"

extern volatile uint32_t uwTick;

#define SYS_FREQUENCY 				16000000		// 16Mhz Core Clock
#define SYSTICK_CSR_EN				0				// SYSTICK Enable Register
#define SYSTICK_CSR_TICKINT			1				// SYSTICK Exception Enable
#define SYSTICK_CSR_CLOCK			2				// SYSTICK Clock Source
#define SYSTICK_CSR_COUNTFLAG		16				// Returns 1 if timer counted to 0

volatile uint32_t myTickCount;

void Systick_Config();
uint32_t PAV_GetTick();
void PAV_IncTick();
void PAV_Delay(uint32_t delay);

#endif /* DRIVERS_INC_STM32F7XX_PAV_SYSTICK_H_ */
