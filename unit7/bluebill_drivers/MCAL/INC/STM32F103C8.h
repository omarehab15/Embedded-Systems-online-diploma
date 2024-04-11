/*
 * STN32F103C8.h
 *
 *  Created on: Jan 27, 2024
 *      Author: omar
 */

#ifndef STM32F103C8_H_
#define STM32F103C8_H_

/******************************************************************************
 *                               INCLUDES			                          *
 *******************************************************************************/
#include "stdlib.h"
#include "stdint.h"

/******************************************************************************
 *                         Base addresses for Memories		                  *
 *******************************************************************************/
#define FLASH_Memory_Base							0x08000000UL
#define System_Memory_Base							0x1FFFF000UL
#define SRAM_Memory_Base							0x20000000UL
#define Peripherals_Base							0x40000000UL
#define Cortex_M3_internal_Peripherals_Base			0xE0000000UL

/******************************************************************************
 *                       Base addresses for AHB Peripherals		              *
 *******************************************************************************/
//-----------------------------
//RCC
//-----------------------------
#define RCC_BASE              				(Peripherals_Base + 0x00021000UL)

//-----------------------------
//DMA
//-----------------------------
#define DMA1_BASE							(Peripherals_Base + 0x00020000UL)
#define DMA2_BASE							(Peripherals_Base + 0x00020400UL)

//-----------------------------
//RCC
//-----------------------------
#define CRC_BASE              				(Peripherals_Base + 0x00021000UL)

/******************************************************************************
 *                       Base addresses for APB2 Peripherals		              *
 *******************************************************************************/
//-----------------------------
//GPIO
//-----------------------------
#define AFIO_BASE							(Peripherals_Base + 0x00010000UL)
#define EXTI_BASE							(Peripherals_Base + 0x00010400UL)
#define GPIOA_BASE							(Peripherals_Base + 0x00010800UL)
#define GPIOB_BASE							(Peripherals_Base + 0x00010C00UL)
#define GPIOC_BASE							(Peripherals_Base + 0x00011000UL)
#define GPIOD_BASE							(Peripherals_Base + 0x00011400UL)
#define GPIOE_BASE							(Peripherals_Base + 0x00011800UL)

//-----------------------------
//ADC
//-----------------------------
#define ADC1_BASE							(Peripherals_Base + 0x00012400UL)
#define ADC2_BASE							(Peripherals_Base + 0x00012800UL)

//-----------------------------
//Advanced_control_timers
//-----------------------------
#define TIM1_BASE							(Peripherals_Base + 0x00012C00UL)

//-----------------------------
//SPI1
//-----------------------------
#define SPI1_BASE							(Peripherals_Base + 0x00013000UL)

//-----------------------------
//SPI1
//-----------------------------
#define USART1_BASE							(Peripherals_Base + 0x00013800UL)


#define NVIC_BASE							(Cortex_M3_internal_Peripherals_Base + 0x0000E100UL)
/******************************************************************************
 *                      Base addresses for APB1 Peripherals		              *
 *******************************************************************************/
//-----------------------------
//General_purpose_timers
//-----------------------------
#define TIM2_BASE							(Peripherals_Base + 0x00000000UL)
#define TIM3_BASE							(Peripherals_Base + 0x00000400UL)
#define TIM4_BASE							(Peripherals_Base + 0x00000800UL)

//-----------------------------
//RTC
//-----------------------------
#define RTC_BASE							(Peripherals_Base + 0x00002800UL)

//-----------------------------
//WatchDog
//-----------------------------
//WWDG
#define WWDG_BASE							(Peripherals_Base + 0x00002C00UL)
//IWDG
#define IWDG_BASE							(Peripherals_Base + 0x00003000UL)

//-----------------------------
//SPI2/I2C
//-----------------------------
#define SPI2_BASE							(Peripherals_Base + 0x00003800UL)

//-----------------------------
//USART
//-----------------------------
#define USART2_BASE							(Peripherals_Base + 0x00004400UL)
#define USART3_BASE							(Peripherals_Base + 0x00004800UL)

//-----------------------------
//I2C
//-----------------------------
#define I2C1_BASE							(Peripherals_Base + 0x00005400UL)
#define I2C2_BASE							(Peripherals_Base + 0x00005800UL)

//-----------------------------
//CAN
//-----------------------------
#define CAN1_BASE							(Peripherals_Base + 0x00006400UL)
#define CAN2_BASE							(Peripherals_Base + 0x00006800UL)

//-----------------------------
//CAN
//-----------------------------
#define DAC_BASE							(Peripherals_Base + 0x00007400UL)

/******************************************************************************
 *                     		   Peripheral register		           		      *
 *******************************************************************************/

//-----------------------------
//RCC Registers
//-----------------------------
typedef struct {
	volatile uint32_t CR ;
	volatile uint32_t CFGR ;
	volatile uint32_t CIR ;
	volatile uint32_t APB2RSTR ;
	volatile uint32_t APB1RSTR ;
	volatile uint32_t AHBENR ;
	volatile uint32_t APB2ENR ;
	volatile uint32_t APB1ENR ;
	volatile uint32_t BDCR ;
	volatile uint32_t CSR ;

}RCC_TypeDef;

//-----------------------------
//GPIO Registers
//-----------------------------
typedef struct {
	volatile uint32_t CRL ;
	volatile uint32_t CRH ;
	volatile uint32_t IDR ;
	volatile uint32_t ODR ;
	volatile uint32_t BSRR ;
	volatile uint32_t BRR ;
	volatile uint32_t LCKR ;

}GPIO_TypeDef;

//-----------------------------
//AFIO Registers
//-----------------------------
typedef struct {
	volatile uint32_t EVCR ;
	volatile uint32_t MAPR ;
	volatile uint32_t CR1 ;
	volatile uint32_t CR2 ;
	volatile uint32_t CR3 ;
	volatile uint32_t CR4 ;
	volatile uint32_t MAPR2 ;

}AFIO_TypeDef;

//-----------------------------
//EXTI Registers
//-----------------------------
typedef struct {
	volatile uint32_t IMR ;
	volatile uint32_t EMR ;
	volatile uint32_t RTSR ;
	volatile uint32_t FTSR ;
	volatile uint32_t SWIER ;
	volatile uint32_t PR ;

}EXTI_TypeDef;

//-----------------------------
//NVIC Registers
//-----------------------------
typedef struct
{
	volatile uint32_t ISER0 	;
	volatile uint32_t ISER1 	;
	volatile uint32_t ISER2 	;
	volatile uint32_t RESERVED0 ;
	volatile uint32_t ICER0 	;
	volatile uint32_t ICER1		;
	volatile uint32_t ICER2		;

}NVIC_TypeDef;

//-----------------------------
//USART Registers
//-----------------------------
typedef struct
{
	volatile uint32_t SR	;
	volatile uint32_t DR 	;
	volatile uint32_t BRR 	;
	volatile uint32_t CR1   ;
	volatile uint32_t CR2   ;
	volatile uint32_t CR3   ;
	volatile uint32_t GTPR  ;

}USART_TypeDef;

//-----------------------------
//SPI Registers
//-----------------------------
typedef struct
{
	volatile uint32_t CR1	 	;
	volatile uint32_t CR2 	 	;
	volatile uint32_t SR 	 	;
	volatile uint32_t DR 	 	;
	volatile uint32_t CRCPR  	;
	volatile uint32_t RXCRCR 	;
	volatile uint32_t TXCRCR    ;
	volatile uint32_t I2SCFGR   ;
	volatile uint32_t I2SPR		;

}SPI_TypeDef;
/******************************************************************************
 *                     		   Peripheral Instants:		           		      *
 *******************************************************************************/
#define GPIOA      					((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB      					((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC      					((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD      					((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE      					((GPIO_TypeDef *)GPIOE_BASE)
#define RCC      					((RCC_TypeDef  *)RCC_BASE)
#define EXTI      					((EXTI_TypeDef *)EXTI_BASE)
#define AFIO      					((AFIO_TypeDef *)AFIO_BASE)
#define NVIC      					((NVIC_TypeDef *)NVIC_BASE)
#define USART1     					((USART_TypeDef *)USART1_BASE)
#define USART2     					((USART_TypeDef *)USART2_BASE)
#define USART3     					((USART_TypeDef *)USART3_BASE)
#define SPI1     					((SPI_TypeDef *)SPI1_BASE)
#define SPI2     					((SPI_TypeDef *)SPI2_BASE)
#endif /* STM32F103C8_H_ */
