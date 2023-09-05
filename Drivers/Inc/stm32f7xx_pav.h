/*
 * stm32f7xx_def.h
 *
 *  Created on: Aug 21, 2023
 *      Author: Yavuz
 */

#ifndef DRIVERS_INC_STM32F7XX_PAV_H_
#define DRIVERS_INC_STM32F7XX_PAV_H_

#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

typedef enum
{
/******  Cortex-M7 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M7 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M7 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M7 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M7 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M7 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M7 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M7 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interrupt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare Interrupt                                    */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FMC_IRQn                    = 48,     /*!< FMC global Interrupt                                              */
  SDMMC1_IRQn                 = 49,     /*!< SDMMC1 global Interrupt                                           */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  RNG_IRQn                    = 80,     /*!< RNG global interrupt                                              */
  FPU_IRQn                    = 81,     /*!< FPU global interrupt                                              */
  UART7_IRQn                  = 82,     /*!< UART7 global interrupt                                            */
  UART8_IRQn                  = 83,     /*!< UART8 global interrupt                                            */
  SPI4_IRQn                   = 84,     /*!< SPI4 global Interrupt                                             */
  SPI5_IRQn                   = 85,     /*!< SPI5 global Interrupt                                             */
  SPI6_IRQn                   = 86,     /*!< SPI6 global Interrupt                                             */
  SAI1_IRQn                   = 87,     /*!< SAI1 global Interrupt                                             */
  LTDC_IRQn                   = 88,     /*!< LTDC global Interrupt                                             */
  LTDC_ER_IRQn                = 89,     /*!< LTDC Error global Interrupt                                       */
  DMA2D_IRQn                  = 90,     /*!< DMA2D global Interrupt                                            */
  SAI2_IRQn                   = 91,     /*!< SAI2 global Interrupt                                             */
  QUADSPI_IRQn                = 92,     /*!< Quad SPI global interrupt                                         */
  LPTIM1_IRQn                 = 93,     /*!< LP TIM1 interrupt                                                 */
  CEC_IRQn                    = 94,     /*!< HDMI-CEC global Interrupt                                         */
  I2C4_EV_IRQn                = 95,     /*!< I2C4 Event Interrupt                                              */
  I2C4_ER_IRQn                = 96,     /*!< I2C4 Error Interrupt                                              */
  SPDIF_RX_IRQn               = 97,     /*!< SPDIF-RX global Interrupt                                         */
  DFSDM1_FLT0_IRQn	          = 99,     /*!< DFSDM1 Filter 0 global Interrupt                                  */
  DFSDM1_FLT1_IRQn	          = 100,    /*!< DFSDM1 Filter 1 global Interrupt                                  */
  DFSDM1_FLT2_IRQn	          = 101,    /*!< DFSDM1 Filter 2 global Interrupt                                  */
  DFSDM1_FLT3_IRQn	          = 102,    /*!< DFSDM1 Filter 3 global Interrupt                                  */
  SDMMC2_IRQn                 = 103,    /*!< SDMMC2 global Interrupt                                           */
  CAN3_TX_IRQn                = 104,    /*!< CAN3 TX Interrupt                                                 */
  CAN3_RX0_IRQn               = 105,    /*!< CAN3 RX0 Interrupt                                                */
  CAN3_RX1_IRQn               = 106,    /*!< CAN3 RX1 Interrupt                                                */
  CAN3_SCE_IRQn               = 107,    /*!< CAN3 SCE Interrupt                                                */
  JPEG_IRQn                   = 108,    /*!< JPEG global Interrupt                                             */
  MDIOS_IRQn                  = 109     /*!< MDIO Slave global Interrupt                                       */
} IRQn_Type;

/* Base Adress of Peripheral Bus AHBx and APBx (RM pg. 77 ) */
#define APB1_BASEADDR 0x40000000
#define APB2_BASEADDR 0x40010000
#define AHB1_BASEADDR 0x40020000
#define AHB2_BASEADDR 0x50000000
#define AHB3_BASEADDR 0xA0000000

/* Base Address of AHB1 Peripherals (RM pg. 78 ) */
#define GPIOA_BASEADDR AHB1_BASEADDR
#define GPIOB_BASEADDR (AHB1_BASEADDR + 0x0400)
#define GPIOC_BASEADDR (AHB1_BASEADDR + 0x0800)
#define GPIOD_BASEADDR (AHB1_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR (AHB1_BASEADDR + 0x1000)
#define GPIOF_BASEADDR (AHB1_BASEADDR + 0x1400)
#define GPIOG_BASEADDR (AHB1_BASEADDR + 0x1800)
#define GPIOH_BASEADDR (AHB1_BASEADDR + 0x1C00)

#define CRC_BASEADDR   (AHB1_BASEADDR + 0x3000)
#define RCC_BASEADDR   (AHB1_BASEADDR + 0x3800)

#define DMA1_BASEADDR  (AHB1_BASEADDR + 0x6000)
#define DMA2_BASEADDR  (AHB1_BASEADDR + 0x6400)

#define ETH_BASEADDR  (AHB1_BASEADDR + 0x8000)

#define USB_OTG_HS_BASEADDR		(0x40040000)

/* Base Address of AHB2 Peripherals (RM pg. 77)*/
#define USB_OTG_FS_BASEADDR		(0x50000000)


/* Base Address of APB1 Peripheral Bus (RM pg. 80) */
#define TIM2_BASEADDR		(APB1_BASEADDR)
#define TIM3_BASEADDR		(APB1_BASEADDR + 0x0400)
#define TIM4_BASEADDR		(APB1_BASEADDR + 0x0800)
#define TIM5_BASEADDR		(APB1_BASEADDR + 0x0C00)
#define TIM6_BASEADDR		(APB1_BASEADDR + 0x1000)
#define TIM7_BASEADDR		(APB1_BASEADDR + 0x1400)
#define TIM12_BASEADDR		(APB1_BASEADDR + 0x1800)
#define TIM13_BASEADDR		(APB1_BASEADDR + 0x1C00)
#define TIM14_BASEADDR		(APB1_BASEADDR + 0x2000)

#define WWDG_BASEADDR		(APB1_BASEADDR + 0x2C00)
#define IWDG_BASEADDR		(APB1_BASEADDR + 0x3000)

#define CAN1_BASEADDR		(APB1_BASEADDR + 0x6400)
#define CAN2_BASEADDR		(APB1_BASEADDR + 0x6800)
#define CAN3_BASEADDR		(APB1_BASEADDR + 0x3400)

// They share the same base address
#define SPI2_BASEADDR		(APB1_BASEADDR + 0x3800)
#define SPI3_BASEADDR		(APB1_BASEADDR + 0x3C00)
#define I2S_2_BASEADDR		(APB1_BASEADDR + 0x3800)
#define I2S_3_BASEADDR		(APB1_BASEADDR + 0x3C00)

#define USART2_BASEADDR		(APB1_BASEADDR + 0x4400)
#define USART3_BASEADDR		(APB1_BASEADDR + 0x4800)
#define UART4_BASEADDR		(APB1_BASEADDR + 0x4C00)
#define UART5_BASEADDR		(APB1_BASEADDR + 0x5000)
#define UART7_BASEADDR		(APB1_BASEADDR + 0x7800)
#define UART8_BASEADDR		(APB1_BASEADDR + 0x7C00)

#define I2C_1_BASEADDR		(APB1_BASEADDR + 0x5400)
#define I2C_2_BASEADDR		(APB1_BASEADDR + 0x5800)
#define I2C_3_BASEADDR		(APB1_BASEADDR + 0x5C00)
#define I2C_4_BASEADDR		(APB1_BASEADDR + 0x6000)

#define DAC_BASEADDR		(APB1_BASEADDR + 0x7400)

/* Base Address of APB2 Peripheral Bus (RM pg. 79) */
#define TIM1_BASEADDR		APB2_BASEADDR
#define TIM8_BASEADDR		(APB2_BASEADDR + 0x0400)
#define TIM9_BASEADDR		(APB2_BASEADDR + 0x4000)
#define TIM10_BASEADDR		(APB2_BASEADDR + 0x4400)
#define TIM11_BASEADDR		(APB2_BASEADDR + 0x4800)

#define USART1_BASEADDR		(APB2_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2_BASEADDR + 0x1400)

#define ADC_BASEADDR		(APB2_BASEADDR + 0x2000)

#define SPI1_BASEADDR		(APB2_BASEADDR + 0x3000)
#define SPI4_BASEADDR		(APB2_BASEADDR + 0x3400)
#define SPI5_BASEADDR		(APB2_BASEADDR + 0x5000)
#define SPI6_BASEADDR		(APB2_BASEADDR + 0x5400)

#define SYSCFG_BASEADDR		(APB2_BASEADDR + 0x3800)
#define EXTI_BASEADDR		(APB2_BASEADDR + 0x3C00)

#define SCS_BASEADDR        (0xE000E000UL)                            /*!< System Control Space Base Address */
#define NVIC_BASEADDR       (SCS_BASEADDR +  0x0100UL)                /*!< NVIC Base Address */
#define NVIC                ((NVIC_RegTypeDef *)NVIC_BASEADDR)   	  /*!< NVIC configuration struct */

/* GPIO Register Typedef (RM pg. 235) */
typedef struct
{
	__vo uint32_t MODER;		/**< Selects the I/O Mode (Input, Output, Alternate, Analog)
									 Address Offset = 0x00 */

	__vo uint32_t OTYPER;		/**< Selects the Output type (Push-pull, Open-drain)
									 Offset = 0x04 */

	__vo uint32_t OSPEEDR;		/**< Selects the Output Speed
									 Offset = 0x08 */

	__vo uint32_t PUPDR;		/**< Selects the Pull-up or Pull-down
									 Offset = 0x0C */

	__vo uint32_t IDR;			/**< Input Data Register (Read Only)
									 Offset = 0x10 */

	__vo uint32_t ODR;			/**< Output Data Register (Read/Write)
									 Offset = 0x14 */

	__vo uint32_t BSRR;			/**< GPIO port bit set/reset register
									 Offset = 0x18 */

	__vo uint32_t LCKR;			/**< GPIO port configuration lock register
									 Offset = 0x1C */

	__vo uint32_t AFR[2];		/**< GPIO alternate function registers
									 Offset = 0x20 - 0x28 */

}GPIO_RegTypeDef_t;


/* RCC Register Typedef (RM pg. 217) */
typedef struct
{
	__vo uint32_t CR;					/**< Offset = 0x00 */

	__vo uint32_t PLLCFGR;				/**< Offset = 0x04 */

	__vo uint32_t CFGR;					/**< Offset = 0x08 */

	__vo uint32_t CIR;					/**< Offset = 0x0C */

	__vo uint32_t AHB1RSTR;				/**< Offset = 0x10 */

	__vo uint32_t AHB2RSTR;				/**< Offset = 0x14 */

	__vo uint32_t AHB3RSTR;				/**< Offset = 0x18 */

	__vo uint32_t RESERVED_0;			/**< Offset = 0x1C - 0x20 */

	__vo uint32_t APB1RSTR;				/**< Offset = 0x20 */

	__vo uint32_t APB2RSTR;				/**< Offset = 0x24 */

	__vo uint32_t RESERVED_1[2];		/**< Offset = 0x28 - 0x30 */

	__vo uint32_t AHB1ENR;				/**< Offset = 0x30 */

	__vo uint32_t AHB2ENR;				/**< Offset = 0x34 */

	__vo uint32_t AHB3ENR;				/**< Offset = 0x38 */

	__vo uint32_t RESERVED_2;			/**< Offset = 0x3C - 0x40 */

	__vo uint32_t APB1ENR;				/**< Offset = 0x40 */

	__vo uint32_t APB2ENR;				/**< Offset = 0x44 */

	__vo uint32_t RESERVED_3[2];		/**< Offset = 0x48 - 0x50 */

	__vo uint32_t AHB1LPENR;			/**< Offset = 0x50 */

	__vo uint32_t AHB2LPENR;			/**< Offset = 0x54 */

	__vo uint32_t AHB3LPENR;			/**< Offset = 0x58 */

	__vo uint32_t RESERVED_4;			/**< Offset = 0x5C - 0x60 */

	__vo uint32_t APB1LPENR;			/**< Offset = 0x60 */

	__vo uint32_t APB2LPENR;			/**< Offset = 0x64 */

	__vo uint32_t RESERVED_5[2];		/**< Offset = 0x68 - 0x70 */

	__vo uint32_t BDCR;					/**< Offset = 0x70 */

	__vo uint32_t CSR;					/**< Offset = 0x74 */

	__vo uint32_t RESERVED_6[2];		/**< Offset = 0x78 - 0x80 */

	__vo uint32_t SSCGR;				/**< Offset = 0x80 */

	__vo uint32_t PLLI2SCFGR;			/**< Offset = 0x84 */

	__vo uint32_t PLLSAICFGR;			/**< Offset = 0x88 */

	__vo uint32_t DCKCFGR1;				/**< Offset = 0x8C */

	__vo uint32_t DCKCFGR2;				/**< Offset = 0x90 */

}RCC_RegTypeDef_t;


/* EXTI Register Typedef (RM pg. 325) */
typedef struct
{
	__vo uint32_t IMR;			/**< Interrupt Mask Register
									 Address Offset = 0x00 */

	__vo uint32_t EMR;			/**< Event Mask Register
									 Offset = 0x04 */

	__vo uint32_t RTSR;			/**< Rising Trigger Selection Register
									 Offset = 0x08 */

	__vo uint32_t FTSR;			/**< Falling Trigger Selection Register
									 Offset = 0x0C */

	__vo uint32_t SWIER;		/**< Software interrupt event register
									 Offset = 0x10 */

	__vo uint32_t PR;			/**< Pending Register (Read/Write)
									 Offset = 0x14 */

}EXTI_RegTypeDef_t;


/* SYSCFG Register Typedef (RM pg. 244) */
typedef struct
{
	__vo uint32_t MEMRMP;		/**< Offset = 0x00 */

	__vo uint32_t PMC;			/**< Offset = 0x04 */

	__vo uint32_t EXTICR[4];	/**< Offset = 0x08 - 0x14 */

	uint32_t RESERVED;			/**< Offset = 0x18 - 0x1C */

	__vo uint32_t CBR;			/**< Offset = 0x1C */

	__vo uint32_t CMPCR;		/**< Offset = 0x20 */

}SYSCFG_RegTypeDef_t;


/* USART Register Typedef (RM pg. 1285) */
typedef struct
{
	__vo uint32_t CR1;				/**< Control Register 1
										 Offset = 0x00 */

	__vo uint32_t CR2;				/**< Control Register 2
										 Offset = 0x04 */

	__vo uint32_t CR3;				/**< Control Register 3
										 Offset = 0x08 */

	__vo uint32_t BRR;				/**< Baud Rate Register
										 Offset = 0x0C */

	__vo uint32_t GTPR;				/**< Guard Time and Prescalar Register
										 Offset = 0x10 */

	__vo uint32_t RTOR;				/**< Receiver Timeout Register
										 Offset = 0x14 */

	__vo uint32_t RQR;				/**< Request Register
										 Offset = 0x18 */

	__vo uint32_t ISR;				/**< Interrupt and Status Register
										 Offset = 0x1C */

	__vo uint32_t ICR;				/**< Interrupt Clear Register
										 Offset = 0x20 */

	__vo uint32_t RDR;				/**< Receive Data Register
										 Offset = 0x24 */

	__vo uint32_t TDR;				/**< Transmit Data Register
										 Offset = 0x28 */

}USART_RegTypeDef_t;

/*********************************************************************************/
//										SYSTICK
/* PM0253 pg. 213 */
typedef struct
{
	__vo uint32_t	CSR;

	__vo uint32_t	RVR;

	__vo uint32_t	CVR;

	__vo uint32_t	CALIB;

}SYSTICK_RegTypeDef_t;

#define SYSTICK_BASEADDR			0xE000E010
#define SYSTICK						((SYSTICK_RegTypeDef_t *) SYSTICK_BASEADDR)

/*********************************************************************************/

/* Structure type to access the Nested Vectored Interrupt Controller (NVIC). */
typedef struct
{
	__vo uint32_t ISER[8];         /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */

	uint32_t RESERVED0[24];

	__vo uint32_t ICER[8];         /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */

	uint32_t RSERVED1[24];

	__vo uint32_t ISPR[8];         /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */

	uint32_t RESERVED2[24];

	__vo uint32_t ICPR[8U];        /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */

	uint32_t RESERVED3[24];

	__vo uint32_t IABR[8];         /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */

	uint32_t RESERVED4[56];

	__vo uint8_t  IP[240];         /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */

	uint32_t RESERVED5[644];

	__vo  uint32_t STIR;            /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */

} NVIC_RegTypeDef;

/* This is originally defined in core_m7.h file
   This function is called whenever interrupt function is implemented dont touch. */
static void __NVIC_EnableIRQ(IRQn_Type IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    NVIC->ISER[(((uint32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
  }
}

/* Called in PAV_GPIO_Enable_Interrupt() function */
#define NVIC_EnableIRQ              __NVIC_EnableIRQ

/* Define GPIO Ports */
#define GPIOA				((GPIO_RegTypeDef_t *)GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegTypeDef_t *)GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegTypeDef_t *)GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegTypeDef_t *)GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegTypeDef_t *)GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegTypeDef_t *)GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegTypeDef_t *)GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegTypeDef_t *)GPIOH_BASEADDR)

#define RCC					((RCC_RegTypeDef_t *)RCC_BASEADDR)

#define EXTI				((EXTI_RegTypeDef_t *)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_RegTypeDef_t *)SYSCFG_BASEADDR)

/* Define USART */
#define USART1				((USART_RegTypeDef_t *) USART1_BASEADDR)
#define USART2				((USART_RegTypeDef_t *) USART2_BASEADDR)
#define USART3				((USART_RegTypeDef_t *) USART3_BASEADDR)
#define UART4				((USART_RegTypeDef_t *) UART4_BASEADDR)
#define UART5				((USART_RegTypeDef_t *) UART5_BASEADDR)
#define USART6				((USART_RegTypeDef_t *) USART6_BASEADDR)
#define UART7				((USART_RegTypeDef_t *) UART7_BASEADDR)
#define UART8				((USART_RegTypeDef_t *) UART8_BASEADDR)


/* Clock Enable Macro for SYSCFG peripheral */
#define SYSCFG_CLK_EN() (RCC->APB2ENR |= (1 << 14))

/*********************************************** RCC SECTION ***********************************************/
/*!< PPRE1 configuration */
#define RCC_CFGR_PPRE1_Pos                 (10U)
#define RCC_CFGR_PPRE1_Msk                 (0x7UL << RCC_CFGR_PPRE1_Pos)        /*!< 0x00001C00 */
#define RCC_CFGR_PPRE1                     RCC_CFGR_PPRE1_Msk                  /*!< PRE1[2:0] bits (APB1 prescaler) */

/*!< PPRE2 configuration */
#define RCC_CFGR_PPRE2_Pos                 (13U)
#define RCC_CFGR_PPRE2_Msk                 (0x7UL << RCC_CFGR_PPRE2_Pos)        /*!< 0x0000E000 */
#define RCC_CFGR_PPRE2                     RCC_CFGR_PPRE2_Msk                  /*!< PRE2[2:0] bits (APB2 prescaler) */


/*********************************************** USART SECTION ***********************************************/

/* Clock Enable Macros for USARTx/UARTx (RM pg. 188, 195)*/
#define USART1_CLK_EN()		(RCC->APB2ENR |= (1 << 4))
#define USART2_CLK_EN()		(RCC->APB1ENR |= (1 << 17))
#define USART3_CLK_EN()		(RCC->APB1ENR |= (1 << 18))
#define UART4_CLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_CLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_CLK_EN()		(RCC->APB2ENR |= (1 << 5))
#define UART7_CLK_EN()		(RCC->APB1ENR |= (1 << 30))
#define UART8_CLK_EN()		(RCC->APB1ENR |= (1 << 31))

/* Clock Disable Macros for USARTx/UARTx (RM pg. 188, 195)*/
#define USART1_CLK_DIS()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_CLK_DIS()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_CLK_DIS()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_CLK_DIS()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_CLK_DIS()	(RCC->APB2ENR &= ~(1 << 5))
#define UART7_CLK_DIS()		(RCC->APB1ENR &= ~(1 << 30))
#define UART8_CLK_DIS()		(RCC->APB1ENR &= ~(1 << 31))

/* Clock Reset Macros for USARTx/UARTx (RM pg. )*/
#define USART1_CLK_RESET()		(RCC->APB2RSTR |= (1 << 4))
#define USART2_CLK_RESET()		(RCC->APB1RSTR |= (1 << 17))
#define USART3_CLK_RESET()		(RCC->APB1RSTR |= (1 << 18))
#define UART4_CLK_RESET()		(RCC->APB1RSTR |= (1 << 19))
#define UART5_CLK_RESET()		(RCC->APB1RSTR |= (1 << 20))
#define USART6_CLK_RESET()		(RCC->APB2RSTR |= (1 << 5))
#define UART7_CLK_RESET()		(RCC->APB1RSTR |= (1 << 30))
#define UART8_CLK_RESET()		(RCC->APB1RSTR |= (1 << 31))

/*********************************************** GPIO SECTION ***********************************************/

/* Clock Enable Macros for GPIOx Pins (RM pg. 218)*/
#define GPIOA_CLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_CLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_CLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_CLK_EN()		(RCC->AHB1ENR |= (1 << 7))

/* Clock Disable Macros for GPIOx Pins (RM pg. 218)*/
#define GPIOA_CLK_DIS()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DIS()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DIS()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DIS()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DIS()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_CLK_DIS()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_CLK_DIS()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_CLK_DIS()		(RCC->AHB1ENR &= ~(1 << 7))

/* Clock Reset Macros for GPIOx Pins (RM pg. 218)*/
#define GPIOA_CLK_RESET()	(RCC->AHB1RSTR |= (1 << 0))
#define GPIOB_CLK_RESET()	(RCC->AHB1RSTR |= (1 << 1))
#define GPIOC_CLK_RESET()	(RCC->AHB1RSTR |= (1 << 2))
#define GPIOD_CLK_RESET()	(RCC->AHB1RSTR |= (1 << 3))
#define GPIOE_CLK_RESET()	(RCC->AHB1RSTR |= (1 << 4))
#define GPIOF_CLK_RESET()	(RCC->AHB1RSTR |= (1 << 5))
#define GPIOG_CLK_RESET()	(RCC->AHB1RSTR |= (1 << 6))
#define GPIOH_CLK_RESET()	(RCC->AHB1RSTR |= (1 << 7))

/* This macro returns Pin Number to an actual number (GPIO_PIN_3 = 0x0008  ->  3) */
#define GPIO_PIN_TO_NUMBER(x)         ( (x == GPIO_PIN_0)?0:\
										(x == GPIO_PIN_1)?1:\
										(x == GPIO_PIN_2)?2:\
										(x == GPIO_PIN_3)?3:\
								        (x == GPIO_PIN_4)?4:\
								        (x == GPIO_PIN_5)?5:\
								        (x == GPIO_PIN_6)?6:\
										(x == GPIO_PIN_7)?7:\
										(x == GPIO_PIN_8)?8:\
										(x == GPIO_PIN_9)?9:\
										(x == GPIO_PIN_10)?10:\
										(x == GPIO_PIN_11)?11:\
										(x == GPIO_PIN_12)?12:\
										(x == GPIO_PIN_13)?13:\
										(x == GPIO_PIN_14)?14:\
								        (x == GPIO_PIN_15)?15:0)

/* This macro returns Port Number to an actual number (GPIOB -> 1) */
#define GPIO_PORT_TO_NUMBER(x)         ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
										(x == GPIOH)?7:0)

//#include "stm32f7xx_pav_gpio.h"
//#include "stm32f7xx_pav_usart.h"

#endif /* DRIVERS_INC_STM32F7XX_PAV_H_ */
