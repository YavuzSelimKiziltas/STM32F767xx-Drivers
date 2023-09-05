/*
 * stm32f7xx_pav_usart.c
 *
 *  Created on: Aug 24, 2023
 *      Author: Yavuz
 */

#include "stm32f7xx_pav_usart.h"
#include "stddef.h"
#define ENABLE 1

uint8_t USART_GetFlagStatus(uint32_t usartRegister, uint8_t flag)
{
	if(usartRegister & (1 << flag))
		return 1;

	else
		return 0;
}

/**
 * @brief Waits until flag si set or timeout is over
 * @retval If it is okey returns 1, If timeout has reached returns -1
 */
uint8_t WaitOnFlagUntilTimeout(USART_RegTypeDef_t *USARTx, uint8_t flag, uint8_t flagStatus, uint32_t tickStart, uint32_t Timeout)
{
	while((USARTx->ISR & (1 << flag)) == flagStatus)
	{
		if(((PAV_GetTick() - tickStart) > Timeout) || (Timeout == 0))
		{
			/* Disable TXE, RXNE, PE and ERR (Frame error, noise error, overrun error)
		    interrupts for the interrupt process */
	        USARTx->CR1 &= ~((1 << USART_CR1_RXNEIE) | (1 << USART_CR1_PEIE) | (1 << USART_CR1_TXEIE));
	        USARTx->CR3 &= ~(1 << USART_CR3_EIE);

			return -1;
		}

	}

	return 1;
}


/**
 * @brief Enable or Disable USART Clock Bus
 * @param USARTx is the USART/UART port selected where x can be (1...8)
 * @param ClockState can be either 1 or 0
 * @retval None
 */
void PAV_USART_ClockCmd(USART_RegTypeDef_t *USARTx, uint8_t ClockState)
{
	if(ClockState == 1)
	{
		if(USARTx == USART1)
			USART1_CLK_EN();

		else if(USARTx == USART2)
			USART2_CLK_EN();

		else if(USARTx == USART3)
			USART3_CLK_EN();

		else if(USARTx == UART4)
			UART4_CLK_EN();

		else if(USARTx == UART5)
			UART5_CLK_EN();

		else if(USARTx == USART6)
			USART6_CLK_EN();

		else if(USARTx == UART7)
			UART7_CLK_EN();

		else if(USARTx == UART8)
			UART8_CLK_EN();
	}

	else if(ClockState == 0)
	{
		if(USARTx == USART1)
			USART1_CLK_DIS();

		else if(USARTx == USART2)
			USART2_CLK_DIS();

		else if(USARTx == USART3)
			USART3_CLK_DIS();

		else if(USARTx == UART4)
			UART4_CLK_DIS();

		else if(USARTx == UART5)
			UART5_CLK_DIS();

		else if(USARTx == USART6)
			USART6_CLK_DIS();

		else if(USARTx == UART7)
			UART7_CLK_DIS();

		else if(USARTx == UART8)
			UART8_CLK_DIS();
	}
}

/**
 * @brief Calculates the baud rate to write in BRR register
 * @param USARTx is the USART/UART port selected where x can be (1...8)
 * @param ClockState can be either 1 or 0
 * @retval None
 */
static void USART_SetBaudRate(USART_RegTypeDef_t *USARTx, uint32_t BaudRate)
{
	uint32_t fClock;			// Clock Frequency
	uint16_t USARTDIV;			// Be Careful!!! USARTDIV must be >= 0x10
	uint16_t BRR;

	if(USARTx == USART1 || USARTx == USART6)
	{
		fClock = HAL_RCC_GetPCLK2Freq();
	}
	else
	{
		fClock = HAL_RCC_GetPCLK1Freq();
	}

	// Check if OVER8 = 1
	if(USARTx->CR1 & (1 << 15))
	{
		USARTDIV = (2 * fClock) / BaudRate;

		BRR = USARTDIV & (0xFFF0);				// BRR[15:4] = USARTDIV[15:4]
		USARTDIV = (USARTDIV >> 1);				// USARTDIV >> 1
		BRR = BRR | ((USARTDIV & 0x0007));		// BRR[3:0] = USARTDIV[3:0], BRR[3] = 0
	}
	else
	{
		USARTDIV = fClock / BaudRate;
		BRR = USARTDIV;
	}

	USARTx->BRR |= BRR;
}


/**
 * @brief USART Initialization function
 * @param USARTx is the USART/UART port selected where x can be (1...8)
 * @param USART_Config holds the configuration settings
 * @retval None
 */
void PAV_USART_Init(USART_RegTypeDef_t *USARTx, USART_ConfigTypeDef_t *USART_Config)
{
	uint32_t tempReg = 0x00;

	// Enable the specified clock bus
	PAV_USART_ClockCmd(USARTx, ENABLE);

	/**************** BRR Settings ****************/

	USART_SetBaudRate(USARTx, USART_Config->BaudRate);

	/**************** CR1 Settings ****************/
	tempReg = USART_Config->Parity 	   	|
			  USART_Config->WordLength 	|
			  USART_Config->OverSampling;

	tempReg &= ~((1 << USART_CR1_RE) | (1 << USART_CR1_TE));
	USARTx->CR1 = tempReg;

	/**************** CR2 Settings ****************/
	tempReg = 0x00;	// Clear the temporary register

	if(USART_Config->SynchronousType == ASYNCHRONOUS)
	{
		tempReg = USART_Config->StopBit;
	}
	else if(USART_Config->SynchronousType == SYNCHRONOUS)
	{
		tempReg = USART_Config->StopBit		 |
				  USART_Config->ClockLastBit |
				  USART_Config->ClockPhase	 |
				  USART_Config->ClockPolarity;
	}

	USARTx->CR2 = tempReg;

	/**************** CR3 Settings ****************/
	tempReg = 0x00;	// Clear the temporary register

	tempReg = USART_Config->FlowControl;
	//tempReg |= (1 << USART_CR3_OVRDIS);
	//tempReg |= (1 << USART_CR3_ONEBIT);

	USARTx->CR3 = tempReg;

	/**************** Enable USART/UART ****************/
	
	if(USART_Config->SynchronousType == ASYNCHRONOUS)
	{
		USARTx->CR2	&= ~( (1 << USART_CR2_CLKEN) | (1 << USART_CR2_LINEN) );
		USARTx->CR3 &= ~( (1 << USART_CR3_SCEN) | (1 << USART_CR3_HDSEL) | (1 << USART_CR3_IREN) );
	}

	USARTx->CR1 |= (1 << USART_CR1_UE);
}

/**
 * @brief USART Deinitialization function
 * @param USARTx is the USART/UART port selected where x can be (1...8)
 * @retval None
 */
void PAV_USART_DeInit(USART_RegTypeDef_t *USARTx)
{
	if(USARTx == USART1)
		USART1_CLK_RESET();

	else if(USARTx == USART2)
		USART2_CLK_RESET();

	else if(USARTx == USART3)
		USART3_CLK_RESET();

	else if(USARTx == UART4)
		UART4_CLK_RESET();

	else if(USARTx == UART5)
		UART5_CLK_RESET();

	else if(USARTx == USART6)
		USART6_CLK_RESET();

	else if(USARTx == UART7)
		UART7_CLK_RESET();

	else if(USARTx == UART8)
		UART8_CLK_RESET();
}

/**
 * @brief USART Transmit function in Polling Mode (RM pg. 1249)
 * @param USARTx is the USART/UART port selected where x can be (1...8)
 * @param pData is the data we want to send
 * @param len is the length of pData
 * @retval None
 */
void PAV_USART_Transmit(USART_RegTypeDef_t *USARTx, uint8_t *pTxBuffer, uint16_t len, uint32_t Timeout)
{
	uint32_t tickStart;
	uint8_t  *pData8Bits;
	uint16_t *pData16Bits;

	tickStart = PAV_GetTick();

	// Check if word length is 9 bits and Parity is None
	if((USARTx->CR1 & (1 << USART_CR1_M0)) & !(USARTx->CR1 & (1 << USART_CR1_M1)) & !(USARTx->CR1 & (1 << USART_CR1_PCE)))
	{
		pData8Bits = NULL;
		pData16Bits = (uint16_t *) pTxBuffer;
	}
	else
	{
		pData16Bits = NULL;
		pData8Bits  = pTxBuffer;
	}

	for(uint16_t i = len; i > 0; i--)
	{
		// Wait until USART->ISR-> TXE == 1
		if(WaitOnFlagUntilTimeout(USARTx, USART_ISR_TXE, 0, tickStart, Timeout) == -1)
		{
			return;
		}

		if(pData8Bits == NULL)
		{
			USARTx->TDR = ((*pData16Bits++) & 0x01FF);
		}
		else
		{
			USARTx->TDR = ((*pData8Bits++) & 0xFF);
		}

	}

	// Wait until USART->ISR-> TC == 1
	while(!USART_GetFlagStatus(USARTx->ISR, USART_ISR_TC));

}


/**
 * @brief USART Receive function in Polling Mode
 * @param USARTx is the USART/UART port selected where x can be (1...8)
 * @param pData is the data we receive
 * @param len is the length of pData
 * @retval None
 */
void PAV_USART_Receive(USART_RegTypeDef_t *USARTx, uint8_t *pRxBuffer, uint16_t len, uint16_t Timeout)
{
	uint32_t tickStart;
	uint8_t  *pData8Bits;
	uint16_t *pData16Bits;

	// Start the timer
	tickStart = PAV_GetTick();

	// Check if word length is 9 bits and Parity is None
	if((USARTx->CR1 & (1 << USART_CR1_M0)) & !(USARTx->CR1 & (1 << USART_CR1_M1)) & !(USARTx->CR1 & (1 << USART_CR1_PCE)))
	{
		pData8Bits = NULL;
		pData16Bits = (uint16_t *) pRxBuffer;
	}
	else
	{
		pData16Bits = NULL;
		pData8Bits  = pRxBuffer;
	}


	for (uint16_t i = 0; i < len; i++)
	{

	    // Wait for data to be received
	    if (WaitOnFlagUntilTimeout(USARTx, USART_ISR_RXNE, 0, tickStart, Timeout) == -1) {
	        return;
	    }


	    if (pData8Bits != NULL) {
	        pRxBuffer[i] = USARTx->RDR;
	    }
	    else if (pData16Bits != NULL)
	    {
	        pRxBuffer[i] = USARTx->RDR & 0x01FF;
	    }
	}

}


void PAV_USART_Start_Receive_IT(USART_RegTypeDef_t *USARTx)
{
	// If Parity is Enabled
	if(USARTx->CR1 & (1 << USART_CR1_PCE))
	{
		USARTx->CR1 |= (1 << USART_CR1_PEIE);
		USARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}
	else
	{
		USARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	NVIC_EnableIRQ(USART_TO_IRQ_NUMBER(USARTx));
}

void PAV_USART_Start_Transmit_IT(USART_RegTypeDef_t *USARTx)
{
	USARTx->CR1 |= (1 << USART_CR1_TXEIE);

	NVIC_EnableIRQ(USART_TO_IRQ_NUMBER(USARTx));
}


void PAV_USART_Receive_IT(USART_RegTypeDef_t *USARTx, uint8_t *pRxBuffer, uint16_t len)
{

	uint8_t  *pData8Bits;
	uint16_t *pData16Bits;


	// Check if word length is 9 bits and Parity is None
	if((USARTx->CR1 & (1 << USART_CR1_M0)) & !(USARTx->CR1 & (1 << USART_CR1_M1)) & !(USARTx->CR1 & (1 << USART_CR1_PCE)))
	{
		pData8Bits = NULL;
		pData16Bits = (uint16_t *) pRxBuffer;
	}
	else
	{
		pData16Bits = NULL;
		pData8Bits  = pRxBuffer;
	}


	for (uint16_t i = 0; i < len; i++)
	{

	   while(!(USARTx->ISR & (1 << USART_ISR_RXNE)));

		// Check for overrun error
		if (USART_GetFlagStatus(USARTx->ISR, USART_ISR_ORE)) {
			// Clear the overrun error flag
			USARTx->ICR |= (1 << USART_ICR_ORECF);
			return; // Handle the error, e.g., return or break from the loop
		}

		if (pData8Bits != NULL) {
			pRxBuffer[i] = USARTx->RDR;
		}
		else if (pData16Bits != NULL)
		{
			pRxBuffer[i] = USARTx->RDR & 0x01FF;
		}
	}

}

void PAV_USART_Transmit_IT(USART_RegTypeDef_t *USARTx, uint8_t *pTxBuffer, uint16_t len)
{
	uint8_t  *pData8Bits;
	uint16_t *pData16Bits;


	// Check if word length is 9 bits and Parity is None
	if((USARTx->CR1 & (1 << USART_CR1_M0)) & !(USARTx->CR1 & (1 << USART_CR1_M1)) & !(USARTx->CR1 & (1 << USART_CR1_PCE)))
	{
		pData8Bits = NULL;
		pData16Bits = (uint16_t *) pTxBuffer;
	}
	else
	{
		pData16Bits = NULL;
		pData8Bits  = pTxBuffer;
	}

	for(uint16_t i = len; i > 0; i--)
	{
		while(!(USARTx->ISR & (1 << USART_ISR_TXE)));

		if(pData8Bits == NULL)
		{
			USARTx->TDR = ((*pData16Bits++) & 0x01FF);
		}
		else
		{
			USARTx->TDR = ((*pData8Bits++) & 0xFF);
		}

	}

	// Wait until USART->ISR-> TC == 1
	while(!USART_GetFlagStatus(USARTx->ISR, USART_ISR_TC));

}
