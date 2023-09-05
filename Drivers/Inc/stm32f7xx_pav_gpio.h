/*
 * stm32f7xx_pav_gpio.h
 *
 *  Created on: Aug 21, 2023
 *      Author: Yavuz
 */

#ifndef DRIVERS_INC_STM32F7XX_PAV_GPIO_H_
#define DRIVERS_INC_STM32F7XX_PAV_GPIO_H_

#include "stm32f7xx_pav.h"


/**
  * @brief  GPIO Bit SET and Bit RESET enumeration
  */
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;

/**
  * @brief  GPIO Clock ENABLE DISABLE enumeration
  */
typedef enum
{
  DISABLE = 0,
  ENABLE = 1
}GPIO_ClockState;

/**
  * @brief  Interrupt Edge Selection enumeration
  */
typedef enum
{
  RISING_EDGE,
  FALLING_EDGE,
  RISING_FALLING_EDGE
}Edge_Selection;

#define GPIO_PIN_0                 ((uint16_t)0x0001)  /* Pin 0 selected */
#define GPIO_PIN_1                 ((uint16_t)0x0002)  /* Pin 1 selected */
#define GPIO_PIN_2                 ((uint16_t)0x0004)  /* Pin 2 selected */
#define GPIO_PIN_3                 ((uint16_t)0x0008)  /* Pin 3 selected */
#define GPIO_PIN_4                 ((uint16_t)0x0010)  /* Pin 4 selected */
#define GPIO_PIN_5                 ((uint16_t)0x0020)  /* Pin 5 selected */
#define GPIO_PIN_6                 ((uint16_t)0x0040)  /* Pin 6 selected */
#define GPIO_PIN_7                 ((uint16_t)0x0080)  /* Pin 7 selected */
#define GPIO_PIN_8                 ((uint16_t)0x0100)  /* Pin 8 selected */
#define GPIO_PIN_9                 ((uint16_t)0x0200)  /* Pin 9 selected */
#define GPIO_PIN_10                ((uint16_t)0x0400)  /* Pin 10 selected */
#define GPIO_PIN_11                ((uint16_t)0x0800)  /* Pin 11 selected */
#define GPIO_PIN_12                ((uint16_t)0x1000)  /* Pin 12 selected */
#define GPIO_PIN_13                ((uint16_t)0x2000)  /* Pin 13 selected */
#define GPIO_PIN_14                ((uint16_t)0x4000)  /* Pin 14 selected */
#define GPIO_PIN_15                ((uint16_t)0x8000)  /* Pin 15 selected */
#define GPIO_PIN_ALL               ((uint16_t)0xFFFF)  /* All pins selected */


#define GPIO_MODE_INPUT  	0
#define GPIO_MODE_OUTPUT  	1
#define GPIO_MODE_AF		2
#define GPIO_MODE_ANALOG	3

#define GPIO_SPEED_LOW  	0
#define GPIO_SPEED_MEDIUM  	1
#define GPIO_SPEED_HIGH  	2
#define GPIO_SPEED_ULTRA  	3

#define GPIO_PIN_NOPULL		0
#define GPIO_PIN_PULLUP		1
#define GPIO_PIN_PULLDOWN	2

#define GPIO_MODE_OUTPUT_PP		0
#define GPIO_MODE_OUTPUT_OD		1


/**************************************** Alternate Function Definitions ****************************************/

// @ref to Device Datasheet (stm32f767zi) at page 89 - 102

/* Debug Section */
#define GPIO_AF_SWDIO		0x00	// PA13
#define GPIO_AF_SWCLK		0x00	// PA14

/* USART Section */
#define GPIO_AF4_USART1		4       // (PB14 = TX, PB15 = RX)

#define GPIO_AF7_USART1		7       // (PA8 = CK, PA9 = TX, PA10 = RX, PA11 = CTS, PA12 = RTS)
									// (PB6 = TX, PB7 = RX)

#define GPIO_AF7_USART2		7       // (PD3 = CTS, PD4 = RTS, PD5 = TX, PD6 = RX, PD7 = CK)
									// (PA0 = CTS, PA1 = RTS, PA2 = TX, PA3 = RX, PA4 = CK)

#define GPIO_AF7_USART3		7       // (PD8 = TX, PD9 = RX, PD10 = CK, PD11 = CTS, PD12 = RTS)
									// (PC10 = TX, PC11 = RX, PC12 = CK)
									// (PB10 = TX, PB11 = RX, PB12 = CK, PB13 = CTS, PB14 = RTS)

#define GPIO_AF6_UART4		6		// (PA11 = RX, PA12 = TX)

#define GPIO_AF8_UART4		8		// (PA0 = TX, PA1 = RX, PA15 = RTS)
									// (PB0 = CTS, PB14 = RTS, PB15 = CTS)
									// (PC10 = TX, PC11 = RX)
									// (PD0 = RX, PD1 = TX)
									// (PH13 = TX, PH14 = RX)

#define GPIO_AF1_UART5		1		// (PB5 = RX, PB6 = TX)
#define GPIO_AF7_UART5		7		// (PC8 = RTS, PC9 = CTS)
#define GPIO_AF8_UART5		8		// (PB12 = RX, PB13 = TX) - (PC12 = TX) - (PD2 = RX)

#define GPIO_AF8_USART6		8		// (PC6 = TX, PC7 = RX, PC8 = CK)
									// (PG7 = CK, PG8 = RTS, PG9 = RX, PG12 = RTS, PG13 = CTS, PG14 = TX, PG15 = CTS)

#define GPIO_AF8_UART7		8		// (PE7 = RX, PE8 = TX, PE9 = RTS, PE10 = CTS)
									// (PF6 = RX, PF7 = TX, PF8 = RTS, PF9 = CTS)

#define GPIO_AF12_UART7		12		// (PA15 = TX) - (PB3 = RX, PB4 = TX)

#define GPIO_AF8_UART8		8		// (PD14 = CTS, PD15 = RTS) - (PE0 = RX, PE1 = TX)


/* TIMER Section */
#define GPIO_AF_TIM1		1		// (PA6 = BKIN, PA7 = CH1N, PA8 = CH1, PA9 = CH2, PA10 = CH3, PA11 = CH4, PA12 = ETR)
									// (PB0 = CH2N, PB1 = CH3N, PB12 = BKIN, PB13 = CH1N, PB14 = CH2N, PB15 = CH3N)
									// (PE6 = BKIN2, PE7 = ETR, PE8 = CH1N, PE9 = CH1, PE10 = CH2N, PE11 = CH2, PE12 = CH3N, PE13 = CH3, PE14 = CH4, PE15 = BKIN)

#define GPIO_AF_TIM2		1		// (PA0 = CH1, PA1 = CH2, PA2 = CH3, PA3 = CH4, PA4 = CH5, PA5 = CH1, PA15 = CH1)
									// (PB3 = CH2, PB10 = CH3, PB11 = CH4)

#define GPIO_AF_TIM3		2		// (PA6 = CH1, PA7 = CH2)
									// (PB0 = CH3, PB1 = CH4, PB4 = CH1, PB5 = CH2)
									// (PC6 = CH1, PC7 = CH2, PC8 = CH3, PC9 = CH4)
									// (PD2 = ETR)

#define GPIO_AF_TIM4		2		// (PB6 = CH1, PB7 = CH2, PB8 = CH3, PB9 = CH4)
									// (PD12 = CH1, PD13 = CH2, PD14 = CH3, PD15 = CH4)
									// (PE0 = ETR)

#define GPIO_AF_TIM5		2		// (PA0 = CH1, PA1 = CH2, PA2 = CH4, PA3 = CH4)
									// (PH10 = CH1, PH11 = CH2, PH12 = CH3)

#define GPIO_AF_TIM8		3		// (PA0 = ETR, PA5 = CH1N, PA6 = BKIN, PA7 = CH1N, PA8 = BKIN2)
									// (PB0 = CH2N, PB0 = CH3N, PB14 = CH2N, PB15 = CH3N)
									// (PC6 = CH1, PC7 = CH2, PC8 = CH3, PC9 = CH4)
									// (PH13 = CH1N, PH14 = CH2N, PH15 = CH3N)

#define GPIO_AF_TIM9		3		// (PA2 = CH1, PA3 = CH2) - (PE5 = CH1, PE6 = CH2)

#define GPIO_AF_TIM10		3		// (PB8 = CH1) - (PF6 = CH1)

#define GPIO_AF_TIM11		3		// (PB9 = CH1) - (PF7 = CH1)

#define GPIO_AF_TIM12		9		// (PB14 = CH1, PB15 = CH2) - (PH6 = CH1, PH8 = CH2)

#define GPIO_AF_TIM13		9		// (PA6 = CH1) - (PF8 = CH1)

#define GPIO_AF_TIM14		9		// (PA7 = CH1) - (PF9 = CH1)


/* I2C Section */
#define GPIO_AF4_I2C1		4		// (PB5 = SMBA, PB6 = SCL, PB7 = SDA, PB8 = SCL, PB9 = SDA)

#define GPIO_AF4_I2C2		4		// (PB10 = SCL, PB11 = SDA, PB12 = SMBA)
									// (PF0 = SDA, PF1 = SCL, PF2 = SMBA)
									// (PH4 = SCL, PH5 = SDA, PH6 = SMBA)

#define GPIO_AF4_I2C3		4		// (PA8 = SCL, PA9 = SMBA) - (PC9 = SDA)
									// (PH7 = SCL, PH8 = SDA, PH9 = SMBA)

#define GPIO_AF1_I2C4		1		// (PB8 = SCL, PB9 = SDA)

#define GPIO_AF4_I2C4		4		// (PD11 = SMBA, PD12 = SCL, PD13 = SDA)
									// (PF13 = SMBA, PF14 = SCL, PF15 = SDA)
									// (PH10 = SMBA, PH11 = SCL, PH12 = SDA)

#define GPIO_AF6_I2C4		6		// (PC10 = SCK)

#define GPIO_AF11_I2C4		11		// (PB6 = SCL, PB7 = SDA, PB9 = SMBA)

/* SPI Section */
#define GPIO_AF5_SPI1		5		// (PA4 = NSS, PA5 = SCK, PA6 = MISO, PA7 = MOSI, PA15 = NSS)
									// (PB3 = SCK, PB4 = MISO, PB5 = MOSI) - (PD7 = MISO)
									// (PG9 = MISO, PG10 = NSS, PG11 = SCK)

#define GPIO_AF5_SPI2		5		// (PA9 = SCK, PA11 = NSS, PA12 = SCK)
									// (PB9 = NSS, PB10 = SCK, PB12 = NSS, PB13 = SCK, PB14 = MISO, PB15 = MOSI)
									// (PC1 = MOSI, PC2 = MISO, PC3 = MOSI) - (PD3 = SCK)

#define GPIO_AF7_SPI2		7		// (PB4 = NSS)

#define GPIO_AF5_SPI3		5		// (PD6 = MOSI)

#define GPIO_AF6_SPI3		6		// (PA4 = NSS, PA15 = NSS)
									// (PB3 = SCK, PB4 = MISO, PB5 = MOSI)
									// (PC11 = MISO, PC12 = MOSI)

#define GPIO_AF7_SPI3		7		// (PB2 = MOSI)

#define GPIO_AF5_SPI4		5		// (PE2 = SCK, PE4 = NSS, PE5 = MISO, PE6 = MOSI, PE11 = NSS, P12 = SCK, PE13 = MISO, P14 = MOSI)

#define GPIO_AF5_SPI5		5		// (PF6 = NSS, PF7 = SCK, PF8 = MISO, PF9 = MOSI, PF11 = MOSI)
									// (PH5 = NSS, PH6 = SCK, PH7 = MISO)

#define GPIO_AF5_SPI6		5		// (PG8 = NSS, PG12 = MISO, PG13 = SCK, PG14 = MOSI)

#define GPIO_AF7_SPI6		7		// (PA15 = NSS)

#define GPIO_AF8_SPI6		8		// (PA4 = NSS, PA5 = SCK, PA6 = MISO, PA7 = MOSI)
									// (PB3 = SCK, PB4 = MISO, PB5 = MOSI)


/* CANBus Section */
#define GPIO_AF_CAN1		9		// (PA11 = RX, PA12 = TX) - (PB8 = RX, PB9 = TX)
									// (PD0 = RX, PD1 = TX) - (PH13 = RX, PH14 = TX)

#define GPIO_AF_CAN2		9		// (PB5 = RX, PB6 = TX, PB12 = RX, PB13 = TX)

#define GPIO_AF_CAN3		11		// (PA13 = RX, PA15 = TX) - (PB3 = RX, PB4 = TX)

/* Ethernet */
#define GPIO_AF_ETH			11		// Will be added


typedef struct
{
	uint32_t GPIO_PinNumber;

	uint32_t GPIO_PinMode;

	uint32_t GPIO_PinPuPd;

	uint32_t GPIO_PinSpeed;

	uint32_t GPIO_PinOutputType;

}GPIO_ConfigTypeDef_t;

//*********************************************************************************************************
//							GPIO Configuration and Initialization Functions

void PAV_GPIO_ClockCmd(GPIO_RegTypeDef_t *GPIOx, GPIO_ClockState NewState);
void PAV_GPIO_Init(GPIO_RegTypeDef_t *GPIOx, GPIO_ConfigTypeDef_t *GPIO_Config);
void PAV_GPIO_DeInit(GPIO_RegTypeDef_t *GPIOx);
void PAV_GPIO_Default(GPIO_ConfigTypeDef_t *GPIO_Config);
void PAV_GPIO_AF_Config(GPIO_RegTypeDef_t *GPIOx, uint16_t GPIO_PIN_x, uint16_t GPIO_AF_Mode);

//*********************************************************************************************************
//									GPIO Read/Write Functions

GPIO_PinState PAV_GPIO_ReadPin(GPIO_RegTypeDef_t *GPIOx, uint16_t GPIO_PIN_x);
void PAV_GPIO_WritePin(GPIO_RegTypeDef_t *GPIOx, uint16_t GPIO_PIN_x, GPIO_PinState PinState);
void PAV_GPIO_TogglePin(GPIO_RegTypeDef_t *GPIOx, uint16_t GPIO_PIN_x);

//*********************************************************************************************************
//									GPIO Interrupt Functions

void PAV_GPIO_Config_Interrupt(GPIO_RegTypeDef_t *GPIOx, uint16_t GPIO_PIN_x, Edge_Selection edge);
void PAV_GPIO_Enable_Interrupt(uint16_t GPIO_PIN_x, IRQn_Type IRQn);
void PAV_GPIO_Clear_Interrupt(uint16_t GPIO_PIN_x);

#endif /* DRIVERS_INC_STM32F7XX_PAV_GPIO_H_ */
