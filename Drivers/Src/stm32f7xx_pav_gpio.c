/*
 * stm32f7xx_pav_gpio.c
 *
 *  Created on: Aug 21, 2023
 *      Author: Yavuz
 */

#include "stm32f7xx_pav_gpio.h"

/**
 * @brief Enable or Disable GPIO Clock Bus
 * @param GPIOx is the GPIO port selected where x can be (A...H)
 * @param NewState can be ENABLE or DISABLE
 * @retval None
 */
void PAV_GPIO_ClockCmd(GPIO_RegTypeDef_t *GPIOx, GPIO_ClockState NewState)
{
	if(NewState == ENABLE)
	{
		if(GPIOx == GPIOA)
			GPIOA_CLK_EN();

		else if(GPIOx == GPIOB)
			GPIOB_CLK_EN();

		else if(GPIOx == GPIOC)
			GPIOC_CLK_EN();

		else if(GPIOx == GPIOD)
			GPIOD_CLK_EN();

		else if(GPIOx == GPIOE)
			GPIOE_CLK_EN();

		else if(GPIOx == GPIOF)
			GPIOF_CLK_EN();

		else if(GPIOx == GPIOG)
			GPIOG_CLK_EN();

		else if(GPIOx == GPIOH)
			GPIOH_CLK_EN();
	}

	else
	{
		if(GPIOx == GPIOA)
			GPIOA_CLK_DIS();

		else if(GPIOx == GPIOB)
			GPIOB_CLK_DIS();

		else if(GPIOx == GPIOC)
			GPIOC_CLK_DIS();

		else if(GPIOx == GPIOD)
			GPIOD_CLK_DIS();

		else if(GPIOx == GPIOE)
			GPIOE_CLK_DIS();

		else if(GPIOx == GPIOF)
			GPIOF_CLK_DIS();

		else if(GPIOx == GPIOG)
			GPIOG_CLK_DIS();

		else if(GPIOx == GPIOH)
			GPIOH_CLK_DIS();
	}
}

/**
  * @brief  Inıtialize the GPIO
  * @param  GPIOx where x can be (A..H) to select the GPIO peripheral.
  * @param  GPIO_Config pointer to a GPIO_ConfigTypeDef_t structure that contains
  *         the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void PAV_GPIO_Init(GPIO_RegTypeDef_t *GPIOx, GPIO_ConfigTypeDef_t *GPIO_Config)
{
	uint32_t tempReg;	// Temporary Register
	uint32_t pinNumber = 0x00;
	uint32_t ioPosition = 0x00;
	uint32_t ioCurrent = 0x00;

	// If the user didnt initialize
	PAV_GPIO_ClockCmd(GPIOx, ENABLE);

	for(pinNumber = 0; pinNumber < 16; pinNumber++)
	{
		ioPosition = 1 << pinNumber;
		ioCurrent = (GPIO_Config->GPIO_PinNumber) & ioPosition;

		if(ioCurrent == ioPosition)
		{
			/* MODER Settings */
			GPIOx->MODER &= ~(0x03 << (2 * pinNumber));	// Clear the register before start
			tempReg = (GPIO_Config->GPIO_PinMode << (2 * pinNumber));	// 2 is because MODER[1:0]
			GPIOx->MODER |= tempReg;	// Set the Mode

			/* OTYPER Settings */
			GPIOx->OTYPER &= ~(1 << pinNumber);	// Clear the register before start
			tempReg = (GPIO_Config->GPIO_PinOutputType << pinNumber);
			GPIOx->OTYPER |= tempReg;

			/* OSPEEDR Settings */
			GPIOx->OSPEEDR &= ~(0x03 << (2 * pinNumber));	// Clear the register before start
			tempReg =  (GPIO_Config->GPIO_PinSpeed << (2 * pinNumber)); // 2 is because OSPEEDR[1:0]
			GPIOx->OSPEEDR |= tempReg;

			/* PUPDR Settings */
			GPIOx->PUPDR &= ~(0x03 << (2 * pinNumber));	// Clear the register before start
			tempReg =  (GPIO_Config->GPIO_PinPuPd << (2 * pinNumber)); // 2 is because PUPDR[1:0]
			GPIOx->PUPDR |= tempReg;

		}
	}

}

/**
  * @brief  De-init the GPIO. Resets the AHB1 GPIO Bus
  */
void PAV_GPIO_DeInit(GPIO_RegTypeDef_t *GPIOx)
{
	if(GPIOx == GPIOA)
		GPIOA_CLK_RESET();

	else if(GPIOx == GPIOB)
		GPIOB_CLK_RESET();

	else if(GPIOx == GPIOC)
		GPIOC_CLK_RESET();

	else if(GPIOx == GPIOD)
		GPIOD_CLK_RESET();

	else if(GPIOx == GPIOE)
		GPIOE_CLK_RESET();

	else if(GPIOx == GPIOF)
		GPIOF_CLK_RESET();

	else if(GPIOx == GPIOG)
		GPIOG_CLK_RESET();

	else if(GPIOx == GPIOH)
		GPIOH_CLK_RESET();
}

/**
  * @brief  Starts the GPIO Configurations as default
  * User does'nt need to use this function.
  */
void PAV_GPIO_Default(GPIO_ConfigTypeDef_t *GPIO_Config)
{
	GPIO_Config->GPIO_PinNumber			= GPIO_PIN_ALL;
	GPIO_Config->GPIO_PinMode 			= GPIO_MODE_INPUT;
	GPIO_Config->GPIO_PinSpeed 			= GPIO_SPEED_MEDIUM;
	GPIO_Config->GPIO_PinOutputType 	= GPIO_MODE_OUTPUT_PP;
	GPIO_Config->GPIO_PinPuPd			= GPIO_PIN_NOPULL;
}

/**
  * @brief  GPIO Configuration function for using alternate mode
  * @param  GPIOx where x can be (A..H) to select the GPIO peripheral.
  * @param  GPIO_PIN_x is the pin number where x can be (0...15)
  * @param  GPIO_AF_Mode is one of the AF modes.
  * Check Alternate Function Definitions in stm32f7xx_pav_gpio.h to choose
  */
void PAV_GPIO_AF_Config(GPIO_RegTypeDef_t *GPIOx, uint16_t GPIO_PIN_x, uint16_t GPIO_AF_Mode)
{
	uint8_t AFRy, AFR_Pin;
	uint32_t pinNumber;

	pinNumber = GPIO_PIN_TO_NUMBER(GPIO_PIN_x);

	// Decide if AFRL or AFRH Register
	AFRy = pinNumber / 8;

	// Find the Pin number
	AFR_Pin = pinNumber % 8;

	GPIOx->AFR[AFRy] &= ~(0x0F << ( 4 * AFR_Pin ) ); // Clear the register before start
	GPIOx->AFR[AFRy] |= (GPIO_AF_Mode << ( 4 * AFR_Pin ) );
}

/**
  * @brief  Reads the specified input port pin.
  * @param  GPIOx where x can be (A..H)
  * @param  PinNumber specifies the port bit to read.
  *         This parameter can be (0..15).
  * @retval 0 or 1.
  */
GPIO_PinState PAV_GPIO_ReadPin(GPIO_RegTypeDef_t *GPIOx, uint16_t GPIO_PIN_x)
{
	if( (GPIOx->IDR & GPIO_PIN_x) == GPIO_PIN_x)
		return GPIO_PIN_SET;

	else
		return GPIO_PIN_RESET;
}

/**
  * @brief  Writes to the specified output port pin.
  * @param  GPIOx where x can be (A..H)
  * @param  GPIO_Pin specifies the port bit to write.
  *         This parameter can be (0..15).
  * @param  PinState is either GPIO_PIN_SET or GPIO_PIN_RESET
  * @retval The output port pin value.
  */
void PAV_GPIO_WritePin(GPIO_RegTypeDef_t *GPIOx, uint16_t GPIO_PIN_x, GPIO_PinState PinState)
{
	// Writes 1 to the corresponding Pin number
	if(PinState == GPIO_PIN_SET)
		GPIOx->ODR |= GPIO_PIN_x;

	// Writes 0 to the corresponding Pin number
	else
		GPIOx->ODR &= ~GPIO_PIN_x;
}

/**
  * @brief  Toggles the GPIO_PIN_x pin where x can be (0...15)
  */
void PAV_GPIO_TogglePin(GPIO_RegTypeDef_t *GPIOx, uint16_t GPIO_PIN_x)
{
	GPIOx->ODR ^= GPIO_PIN_x;
}


/**
  * @brief  Configuration function for interrupt
  * @param  GPIOx where x can be (A..H)
  * @param  GPIO_Pin_x is the pin number which interrupt occurs.
  *         This parameter can be (0..15).
  * @param  edge is one of the parameters @ref Edge_Selection enumeration
  * 		(FALLING_EDGE, RISING_EDGE, FALLING_RISING_EDGE)
  * @retval None
  */
void PAV_GPIO_Config_Interrupt(GPIO_RegTypeDef_t *GPIOx, uint16_t GPIO_PIN_x, Edge_Selection edge)
{
	uint32_t extiLineX;
	uint8_t  portNumber, remain, pinNumber;

	// Enable SYSFG Clock
	SYSCFG_CLK_EN();

	// Look at the RM pg.325 for using EXTICR register
	pinNumber  = GPIO_PIN_TO_NUMBER(GPIO_PIN_x);
	portNumber = GPIO_PORT_TO_NUMBER(GPIOx);
	extiLineX  = pinNumber  / 4;
	remain     = pinNumber  % 4;

	SYSCFG->EXTICR[extiLineX] &= ~(0x0F << (4 * remain));		// Clear the selected line
	SYSCFG->EXTICR[extiLineX] |= (portNumber << (4 * remain));	// Apply the changes

	// Apply Edge Selection settings
	if(edge == RISING_EDGE) {
		EXTI->RTSR |= (1 << pinNumber);
		EXTI->FTSR &= ~(1 << pinNumber);	// Clear
	}


	else if(edge == FALLING_EDGE) {
		EXTI->FTSR |= (1 << pinNumber);
		EXTI->RTSR &= ~(1 << pinNumber);	// Clear
	}


	else if(edge == RISING_FALLING_EDGE) {
		EXTI->FTSR |= (1 << pinNumber);
		EXTI->RTSR |= (1 << pinNumber);
	}
}

/**
  * @brief  Enables the interrupt on specified pin and IRQn
  * @param  GPIO_Pin_x is the pin number which interrupt occurs.
  *         This parameter can be (0..15).
  * @param  IRQn must be the same as pin number. For example;
  *         (GPIO_PIN_4 -> EXTI4_IRQn) or (GPIO_PIN_13 -> EXTI15_10_IRQn)
  * @retval None
  */
void PAV_GPIO_Enable_Interrupt(uint16_t GPIO_PIN_x, IRQn_Type IRQn)
{
	uint8_t pinNumber = GPIO_PIN_TO_NUMBER(GPIO_PIN_x);

	//Interrupt mask register is set to 1
	EXTI->IMR |= (1 << pinNumber);

	NVIC_EnableIRQ(IRQn);
}

/**
  * @brief  Clears the Pending Register. Call this in the
  * 		Interrupt handler first.
  * @param  GPIO_Pin_x is the pin number which interrupt occurs.
  *         This parameter can be (0..15).
  * @retval None
  */
void PAV_GPIO_Clear_Interrupt(uint16_t GPIO_PIN_x)
{
	// PR register is cleared by writing 1 to it
	EXTI->PR |= (1 << GPIO_PIN_TO_NUMBER(GPIO_PIN_x));
}

