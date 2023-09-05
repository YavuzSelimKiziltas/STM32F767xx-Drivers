/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f7xx_pav.h"


GPIO_ConfigTypeDef_t GPIO_Config;
int count = 0;

void Gpio_Config()
{
	/* Enable the USER LEDs for writing output*/

	// If you forget to enable clock dont worry it is taken care of in the Init function
	PAV_GPIO_ClockCmd(GPIOB, ENABLE);

	// Multiple Pin option as well as single
	GPIO_Config.GPIO_PinNumber = GPIO_PIN_0 | GPIO_PIN_7 | GPIO_PIN_14;

	// Mode is assigned as Output
	GPIO_Config.GPIO_PinMode = GPIO_MODE_OUTPUT;

	// No Pull up Resistor
	GPIO_Config.GPIO_PinPuPd = GPIO_PIN_NOPULL;

	// Speed is adjusted
	GPIO_Config.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// Output Type is push pull (No special reason)
	GPIO_Config.GPIO_PinOutputType = GPIO_MODE_OUTPUT_PP;

	// Call the Init function to apply configurations !!!
	PAV_GPIO_Init(GPIOB, &GPIO_Config);

	/* Enable USER Button for input same logic applies here */
	PAV_GPIO_ClockCmd(GPIOC, ENABLE);
	GPIO_Config.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Config.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIO_Config.GPIO_PinPuPd = GPIO_PIN_PULLDOWN;
	GPIO_Config.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	PAV_GPIO_Init(GPIOC, &GPIO_Config);

	// Configure Interrupt settings. 3 Edge Selection is available
	PAV_GPIO_Config_Interrupt(GPIOC, GPIO_PIN_13, RISING_EDGE);

	// Enable Interrupt
	PAV_GPIO_Enable_Interrupt(GPIO_PIN_13, EXTI15_10_IRQn);
}

/* Simple Delay function for test purposes */
void delay(){
	for(uint32_t i = 0; i < 200000 ; i++);
}


int main(void)
{
	Gpio_Config();

	PAV_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
	PAV_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	PAV_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);

    /* Loop forever */
	while(1)
	{
		count++;

		delay();
	}
}

/* Carefull !!! This is the Interrupt Handler. Naming must be according to the EXTI line
 * eg. For Pin Number 4 -> EXTI4_IRQn -> void EXTI4_IRQHandler()  */
void EXTI15_10_IRQHandler()
{
	PAV_GPIO_Clear_Interrupt(GPIO_PIN_13);

	// Whenever interrupt occurs reset the count variable and toggle leds
	count = 0;
	PAV_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	PAV_GPIO_TogglePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_14);

}
