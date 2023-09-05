/*
 * stm32f7xx_pav_usart.h
 *
 *  Created on: Aug 24, 2023
 *      Author: Yavuz
 *
 *  ISSUES: While initializing with USART_TX_RX mode it sends data rubbish as a first byte
 */

#ifndef DRIVERS_INC_STM32F7XX_PAV_USART_H_
#define DRIVERS_INC_STM32F7XX_PAV_USART_H_

#include "stm32f7xx_pav.h"

//*********************************************************************************
//								    USART REGISTERS

#define USART_CR1_UE		0		// USART Enable
#define USART_CR1_RE		2		// Receive Enable
#define USART_CR1_TE		3		// Transmit Enable
#define USART_CR1_PS		9		// Parity Selection
#define USART_CR1_PCE		10		// Parity Control Enable
#define USART_CR1_M0		12		// Word Length
#define USART_CR1_M1		28		// Word Length
#define USART_CR1_OVER8		15		// Over Sampling
#define USART_CR1_RXNEIE	5		// Rx interrupt enable
#define USART_CR1_TXEIE		7		// Tx Interrupt Enable
#define USART_CR1_PEIE		8		// PE Interrupt enable


#define USART_CR2_LBCL		8		// Last Bit Enable
#define USART_CR2_CPHA		9		// Clock Phase
#define USART_CR2_CPOL		10		// Clock Polarity
#define USART_CR2_CLKEN		11		// Clock Enable
#define USART_CR2_STOP0		12		// Stop Bit
#define USART_CR2_STOP1		13		// Stop Bit
#define USART_CR2_LINEN		14		// Enable LIN Mode
#define USART_CR2_RTOEN		23		// Enable Receiver Timeout

#define USART_CR3_EIE		0		// Error Interrupt enable
#define USART_CR3_IREN		1		// IrDA Mode Enable
#define USART_CR3_HDSEL 	3		// Half Duplex Selection
#define USART_CR3_SCEN 		5		// SmartCard Mode Enable
#define USART_CR3_RTSE		8		// RTS Enable
#define USART_CR3_CTSE		9		// CTS Enable
#define USART_CR3_ONEBIT	11		// One bit Sample
#define USART_CR3_OVRDIS	12		// Overrun Error Disable


#define USART_ISR_ORE		3		//  Overrun Error
#define USART_ISR_RXNE		5		// RDR not empty
#define USART_ISR_TC		6		// Transmission Complete
#define USART_ISR_TXE		7		// TDR empty
#define USART_ISR_RTOF		11		// Receiver Timeout Flag

#define USART_RQR_RXFRQ		3		// Receive Data flush request

#define USART_ICR_RTOCF		11		// Clear RTOF Flag
#define USART_ICR_ORECF		3		// Clear Overrun Flag

#define USART_TO_IRQ_NUMBER(x)         ( (x == USART1)?37:\
										(x ==  USART2)?38:\
										(x ==  USART3)?39:\
										(x ==  UART4)?52:\
								        (x ==  UART5)?53:\
								        (x ==  USART6)?71:\
								        (x ==  UART7)?82:\
										(x ==  UART8)?83:0)
//**************************************************************************************
//									USART Configurations

/* Some of the Standard BaudRate values USART->BRR ( pg. 1256 - 1257) */
#define USART_BAUD_1200				1200
#define USART_BAUD_2400				2400
#define USART_BAUD_4800				4800
#define USART_BAUD_9600				9600
#define USART_BAUD_14400			14400
#define USART_BAUD_19200			19200
#define USART_BAUD_28800			28800
#define USART_BAUD_38400			38400
#define USART_BAUD_57600			57600
#define USART_BAUD_115200			115200

/* USARTx->CR3-> CTSE, RTSE */
#define FLOW_CONTROL_NONE 			0x00
#define FLOW_CONTROL_CTS_ONLY 		(1 << USART_CR3_CTSE)								// Not Implemented
#define FLOW_CONTROL_RTS_ONLY 		(1 << USART_CR3_RTSE)								// Not Implemented
#define FLOW_CONTROL_CTS_RTS 		(1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE)		// Not Implemented

// USART->CR1
#define USART_WORD_LENGTH_8BIT		0x00					// M0 = 0, M1 = 0
#define USART_WORD_LENGTH_9BIT		(1 << USART_CR1_M0)		// M1 = 0, M0 = 1
#define USART_WORD_LENGTH_7BIT		(1 << USART_CR1_M1)		// M1 = 1, M0 = 0

// USART->CR2
#define USART_STOP_1B				0x00
#define USART_STOP_2B				(1 << USART_CR2_STOP1)

// USART->CR1-> PCE = 0
#define USART_NO_PARITY				0x00
// USART->CR1-> PS-PCE
#define USART_EVEN_PARITY			(1 << USART_CR1_PCE)
#define USART_ODD_PARITY			(1 << USART_CR1_PCE) | (1 << USART_CR1_PS)

// USART->CR1-> OVER8
#define USART_OVERSAMPLING_16		0x00
#define USART_OVERSAMPLING_8		(1 << USART_CR1_OVER8)

// USART->CR1-> TE-CE
#define USART_TX_RX					(1 << USART_CR1_TE) | (1 << USART_CR1_RE)
#define USART_ONLY_TX				(1 << USART_CR1_TE)
#define USART_ONLY_RX				(1 << USART_CR1_RE)


#define ASYNCHRONOUS				0
#define SYNCHRONOUS					1

/* For Synchronous Mode */
// USART->CR2-> CPOL
#define CLOCK_POLARITY_LOW			0x00
#define CLOCK_POLARITY_HIGH			(1 << USART_CR2_CPOL)

// USART->CR2-> LBCL
#define LAST_BIT_DISABLE			0x00
#define LAST_BIT_ENABLE				(1 << USART_CR2_LBCL)

// USART->CR2-> CPHA
#define CLOCK_PHASE_1EDGE			0x00
#define CLOCK_PHASE_2EDGE			(1 << USART_CR2_CPHA)

typedef struct
{
	uint32_t Mode;				/* USART_ONLY_TX, USART_ONLY_RX, USART_TX_RX */

	uint32_t BaudRate;			/* USART_BAUD_x */

	uint32_t WordLength;		/* USART_WORD_LENGTH_x */

	uint32_t Parity;			/* USART_NO_PARITY, USART_EVEN_PARITY, USART_ODD_PARITY*/

	uint32_t StopBit;			/* USART_STOP_1B, USART_STOP_2B */

	uint32_t OverSampling;		/* USART_OVERSAMPLING_8, USART_OVERSAMPLING_16*/

	uint32_t FlowControl;		/* FLOW_CONTROL_NONE, FLOW_CONTROL_CTS_ONLY, FLOW_CONTROL_RTS_ONLY, FLOW_CONTROL_CTS_RTS */

	uint32_t SynchronousType;	/* ASYNCHRONOUS, SYNCHRONOUS */

	uint32_t ClockPolarity;		/* CLOCK_POLARITY_LOW, CLOCK_POLARITY_HIGH */

	uint32_t ClockPhase;		/* CLOCK_PHASE_1EDGE, CLOCK_PHASE_2EDGE */

	uint32_t ClockLastBit;		/* LAST_BIT_ENABLE, LAST_BIT_DISABLE */

}USART_ConfigTypeDef_t;

//*********************************************************************************************************

//							USART Configuration and Initialization Functions
void PAV_USART_ClockCmd(USART_RegTypeDef_t *USARTx, uint8_t ClockState);
void PAV_USART_Init(USART_RegTypeDef_t *USARTx, USART_ConfigTypeDef_t *USART_Config);
void PAV_USART_DeInit(USART_RegTypeDef_t *USARTx);
uint8_t USART_GetFlagStatus(uint32_t usartRegister, uint8_t flag);
uint8_t WaitOnFlagUntilTimeout(USART_RegTypeDef_t *USARTx, uint8_t flag, uint8_t flagStatus, uint32_t tickStart, uint32_t Timeout);

//							USART Transmit and Receive Functions
void PAV_USART_Transmit(USART_RegTypeDef_t *USARTx, uint8_t *pTxBuffer, uint16_t len, uint32_t Timeout);
void PAV_USART_Receive(USART_RegTypeDef_t *USARTx, uint8_t *pRxBuffer, uint16_t len, uint16_t Timeout);

void PAV_USART_Start_Receive_IT(USART_RegTypeDef_t *USARTx);
void PAV_USART_Start_Transmit_IT(USART_RegTypeDef_t *USARTx);
void PAV_USART_Transmit_IT(USART_RegTypeDef_t *USARTx, uint8_t *pTxBuffer, uint16_t len);
void PAV_USART_Receive_IT(USART_RegTypeDef_t *USARTx, uint8_t *pRxBuffer, uint16_t len);


#endif /* DRIVERS_INC_STM32F7XX_PAV_USART_H_ */
