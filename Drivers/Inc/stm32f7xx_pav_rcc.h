/*
 * stm32f7xx_pav_rcc.h
 *
 *  Created on: Aug 26, 2023
 *      Author: Yavuz
 */

#ifndef DRIVERS_INC_STM32F7XX_PAV_RCC_H_
#define DRIVERS_INC_STM32F7XX_PAV_RCC_H_

#include "stm32f7xx_pav.h"

uint32_t SystemCoreClock = 16000000;
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};


uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);



#endif /* DRIVERS_INC_STM32F7XX_PAV_RCC_H_ */
