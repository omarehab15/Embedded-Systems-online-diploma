/*
 * USART.h
 *
 *  Created on: Mar 29, 2024
 *      Author: omar
 */

#ifndef INC_USART_H_
#define INC_USART_H_
/******************************************************************************
*                               INCLUDES			                          *
*******************************************************************************/

#include "stdio.h"
#include "stdlib.h"
#include "STM32F103C8.h"
#include "GPIO.h"
#include "RCC.h"
#include "ISR.h"
/******************************************************************************
*                       	  References Macros         	                  *
*******************************************************************************/
//@ref UART_Mode_define
//Bit 3 TE: Transmitter enable
//This bit enables the transmitter. It is set and cleared by software.
//0: Transmitter is disabled
//1: Transmitter is enabled
//Note: 1: During transmission, a “0” pulse on the TE bit (“0” followed by “1”) sends a preamble
//(idle line) after the current word, except in Smartcard mode.
//2: When TE is set there is a 1 bit-time delay before the transmission starts.
//Bit 2 RE: Receiver enable
//This bit enables the receiver. It is set and cleared by software.
//0: Receiver is disabled
//1: Receiver is enabled and begins searching for a start bit

#define USART_RX_ENABLE     	(uint32_t)(1<<2)
#define USART_TX_ENABLE     	(uint32_t)(1<<3)
#define USART_RX_TX_ENABLE  	(uint32_t)(1<<3 | 1<<2)

//===============================================
//@ref UART_baudRate_define

#define baud_rate_2400         2400
#define baud_rate_9600         9600
#define baud_rate_19200        19200
#define baud_rate_57600        57600
#define baud_rate_115200       115200
#define baud_rate_230400       230400
#define baud_rate_460800       460800
#define baud_rate_921600       921600
#define baud_rate_2250000      2250000
#define baud_rate_4500000      4500000

//===============================================
//Bit 12 M: Word length
//This bit determines the word length. It is set or cleared by software.
//0: 1 Start bit, 8 Data bits, n Stop bit
//1: 1 Start bit, 9 Data bits, n Stop bit

#define word_length_8     (uint32_t)(0)
#define word_length_9     (uint32_t)(1<<12)

//===============================================
//@ref UART_parity_define
//Bit 10 PCE: Parity control enable
//This bit selects the hardware parity control (generation and detection). When the parity
//control is enabled, the computed parity is inserted at the MSB position (9th bit if M=1; 8th bit
//if M=0) and parity is checked on the received data. This bit is set and cleared by software.
//Once it is set, PCE is active after the current byte (in reception and in transmission).
//0: Parity control disabled
//1: Parity control enabled
//Bit 9 PS: Parity selection
//This bit selects the odd or even parity when the parity generation/detection is enabled (PCE
//bit set). It is set and cleared by software. The parity will be selected after the current byte.
//0: Even parity
//1: Odd parity
#define parity_none (uint32_t)(0)
#define parity_even (uint32_t)(1<<10)
#define parity_odd  (uint32_t)(1<<10|1<<9)

//===============================================
//@ref UART_StopBits_define

//Bits 13:12 STOP: STOP bits
//These bits are used for programming the stop bits.
//00: 1 Stop bit
//01: 0.5 Stop bit
//10: 2 Stop bits
//11: 1.5 Stop bit
//The 0.5 Stop bit and 1.5 Stop bit are not available for UART4 & UART5
#define stop_bits_1           (uint32_t)(0)
#define stop_bits_HALF        (uint32_t)(1<<12)
#define stop_bits_2           (uint32_t)(1<<13)
#define stop_bits_ONE_HALF    (uint32_t)(0b11<<12)

//@ref UART_HWFlowCtl_define
#define UART_HWFlowCtl_NONE					0b00
#define UART_HWFlowCtl_RTS					0b01
#define UART_HWFlowCtl_CTS					0b10
#define UART_HWFlowCtl_RTS_CTS				0b11

//==============================================
//@ref interrupt

#define  UART_IRQ_Enable_NONE				(uint32_t)(0)

//Bit 7 TXEIE: TXE interrupt enable
//This bit is set and cleared by software.
//0: Interrupt is inhibited
//1: A USART interrupt is generated whenever TXE=1 in the USART_SR register
#define UART_IRQ_Enable_TXE    				(uint32_t)(1<<7)

//Bit 6 TCIE: Transmission complete interrupt enable
//This bit is set and cleared by software.
//0: Interrupt is inhibited
//1: A USART interrupt is generated whenever TC=1 in the USART_SR register
#define UART_IRQ_Enable_TC            		(uint32_t)(1<<6)

//Bit 5 RXNEIE: RXNE interrupt enable
//This bit is set and cleared by software.
//0: Interrupt is inhibited
//1: A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_SR register
#define UART_IRQ_Enable_RXNE  				(uint32_t)(1<<5)

#define UART_IRQ_Enable_PE					(uint32_t)(1<<8)

typedef enum
{
	enable ,
	disable
}USART_Polling_mechanism_t;

/******************************************************************************
*                       configration structure          	                  *
*******************************************************************************/
typedef struct
{
	uint32_t 	USART_Mode;				//specifies TX/TR Enable/Disable
										//This parameter must be set based on @ref UART_Mode_define

	uint32_t	BaudRate;				//This member configures the UART communication baud rate
										//This parameter must be set based on @ref UART_baudRate_define

	uint8_t		payload_length;			//This member specifies the number of data bits transmitted or received
										//This parameter must be set based on @ref UART_payload_length

	uint8_t		parity;					//This member specifies the parity mode
										//This parameter must be set based on @ref UART_parity_define

	uint8_t		stopbits;				//This member specifies the number of stop bits transmitted
										//This parameter must be set based on @ref UART_StopBits_define

	uint8_t		HWFlowCtl;				//This member specifies the hardware flow control mode is Enable/Disable
										//This parameter must be set based on @ref UART_HWFlowCtl_define

	uint8_t		IRQ_Enable;				//This member Enable or Disable UART IRQ TX/RX
										//This parameter must be set based on @ref UART_IRQ_Enable_define


	void(* P_IRQ_CallBack)(void) ;		//Set the C function which will be called once the IRQ happen

	uint32_t 	Clock ;

}UART_Config;

/******************************************************************************
*                                    APIs			                          *
*******************************************************************************/
void MCAL_UART_Init(USART_TypeDef* USARTx , UART_Config* UART_cnfg);
void MCAL_UART_DeInit(USART_TypeDef* USARTx);

void MCAL_UART_SendData(USART_TypeDef* USARTx , uint16_t* pTxBuffer , USART_Polling_mechanism_t PollingEn);
void MCAL_UART_ReceiveData(USART_TypeDef* USARTx , uint16_t* pRxBuffer , USART_Polling_mechanism_t PollingEn);
void USART_SEND_STRING(USART_TypeDef* USARTx,char* data);

void MCAL_UART_WAIT_TC(USART_TypeDef* USARTx);


#endif /* INC_USART_H_ */
