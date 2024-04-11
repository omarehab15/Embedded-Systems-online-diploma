/*
 * USART.c
 *
 *  Created on: Mar 29, 2024
 *      Author: omar
 */

/******************************************************************************
 *                               INCLUDES			                          *
 *******************************************************************************/
#include "USART.h"

/******************************************************************************
 *                           generic variables			                      *
 *******************************************************************************/
USART_TypeDef* arr[3];
int arr_string_lingh[3];
char * arr_data[3];
uint16_t arr_data_RES[3];

UART_Config* Global_UART_config[3] ={ NULL , NULL , NULL} ;
uint8_t USART_Index = 0 ;

void(* GP_reseve_Callback[3])(void);

/******************************************************************************
 *                           APIS IMPLEMENTATION			                      *
 *******************************************************************************/
void MCAL_UART_Init(USART_TypeDef* USARTx , UART_Config* UART_cnfg)
{
	if(USARTx ==USART1)
	{
		USART_Index = 0 ;

		MCAL_RCC_Peripherals_enable(APB2, RCC_USART1, RCC_Enable);
		MCAL_RCC_Peripherals_enable(APB2, RCC_GPIOA, RCC_Enable);
		//TX
		MCAL_GPIO_Init(GPIOA, PIN9, Output_AF_PP_Mode_Speed10MHZ);
		//RX
		MCAL_GPIO_Init(GPIOA, PIN10, Input_AF);
		arr[0] = USART1 ;

		switch(UART_cnfg->HWFlowCtl)
		{
		case UART_HWFlowCtl_NONE :
		{

		}
		break;
		case UART_HWFlowCtl_RTS :
		{
			MCAL_GPIO_Init(GPIOA, PIN12, Output_AF_PP_Mode_Speed10MHZ);
		}
		break;
		case UART_HWFlowCtl_CTS :
		{
			MCAL_GPIO_Init(GPIOA, PIN11, Input_Flo_Mode);
		}
		break;
		case UART_HWFlowCtl_RTS_CTS :
		{
			MCAL_GPIO_Init(GPIOA, PIN12, Output_AF_PP_Mode_Speed10MHZ);
			MCAL_GPIO_Init(GPIOA, PIN11, Input_Flo_Mode);
		}
		break;
		}
	}
	else if (USARTx ==USART2)
	{
		USART_Index = 1 ;

		MCAL_RCC_Peripherals_enable(APB1, RCC_USART2, RCC_Enable);
		MCAL_RCC_Peripherals_enable(APB2, RCC_GPIOA, RCC_Enable);
		//TX
		MCAL_GPIO_Init(GPIOA, PIN2, Output_AF_PP_Mode_Speed10MHZ);
		//RX
		MCAL_GPIO_Init(GPIOA, PIN3, Input_AF);
		arr[1] = USART2 ;

		switch(UART_cnfg->HWFlowCtl)
		{
		case UART_HWFlowCtl_NONE :
		{

		}
		break;
		case UART_HWFlowCtl_RTS :
		{
			MCAL_GPIO_Init(GPIOA, PIN1, Output_AF_PP_Mode_Speed10MHZ);
		}
		break;
		case UART_HWFlowCtl_CTS :
		{
			MCAL_GPIO_Init(GPIOA, PIN0, Input_Flo_Mode);
		}
		break;
		case UART_HWFlowCtl_RTS_CTS :
		{
			MCAL_GPIO_Init(GPIOA, PIN1, Output_AF_PP_Mode_Speed10MHZ);
			MCAL_GPIO_Init(GPIOA, PIN0, Input_Flo_Mode);
		}
		break;
		}
	}
	else
	{
		USART_Index = 2 ;

		MCAL_RCC_Peripherals_enable(APB1, RCC_USART3, RCC_Enable);
		MCAL_RCC_Peripherals_enable(APB2, RCC_GPIOB, RCC_Enable);
		//TX
		MCAL_GPIO_Init(GPIOB, PIN10, Output_AF_PP_Mode_Speed10MHZ);
		//RX
		MCAL_GPIO_Init(GPIOB, PIN11, Input_AF);
		arr[2] = USART3 ;

		switch(UART_cnfg->HWFlowCtl)
		{
		case UART_HWFlowCtl_NONE :
		{

		}
		break;
		case UART_HWFlowCtl_RTS :
		{
			MCAL_GPIO_Init(GPIOB, PIN14, Output_AF_PP_Mode_Speed10MHZ);
		}
		break;
		case UART_HWFlowCtl_CTS :
		{
			MCAL_GPIO_Init(GPIOA, PIN13, Input_Flo_Mode);
		}
		break;
		case UART_HWFlowCtl_RTS_CTS :
		{
			MCAL_GPIO_Init(GPIOA, PIN14, Output_AF_PP_Mode_Speed10MHZ);
			MCAL_GPIO_Init(GPIOA, PIN13, Input_Flo_Mode);
		}
		break;
		}
	}

	Global_UART_config[USART_Index] = UART_cnfg ;

	//STEP 1
	//	Bit 13 UE: USART enable
	//	When this bit is cleared the USART prescalers and outputs are stopped and the end of the
	//	current
	//	byte transfer in order to reduce power consumption. This bit is set and cleared by software.
	//	0: USART prescaler and outputs disabled
	//	1: USART enabled
	USARTx->CR1 |= (1 << 13);

	//STEP 2
	//	Bit 12 M: Word length
	//	This bit determines the word length. It is set or cleared by software.
	//	0: 1 Start bit, 8 Data bits, n Stop bit
	//	1: 1 Start bit, 9 Data bits, n Stop bit
	USARTx->CR1 |=UART_cnfg->payload_length ;

	//STEP 3
	//	Bits 13:12 STOP: STOP bits
	//	These bits are used for programming the stop bits.
	//	00: 1 Stop bit
	//	01: 0.5 Stop bit
	//	10: 2 Stop bits
	//	11: 1.5 Stop bit
	USARTx->CR1 |=UART_cnfg->stopbits ;

	//STEP 4
	//	Bit 3 TE: Transmitter enable
	//	This bit enables the transmitter. It is set and cleared by software.
	//	0: Transmitter is disabled
	//	1: Transmitter is enabled
	//	Note: 1: During transmission, a “0” pulse on the TE bit (“0” followed by “1”) sends a preamble
	//	(idle line) after the current word, except in Smartcard mode.
	//	2: When TE is set there is a 1 bit-time delay before the transmission starts.
	//	Bit 2 RE: Receiver enable
	//	This bit enables the receiver. It is set and cleared by software.
	//	0: Receiver is disabled
	//	1: Receiver is enabled and begins searching for a start bit
	USARTx->CR1 |=UART_cnfg->USART_Mode ;

	//STEP 5
	//	Bit 10 PCE: Parity control enable
	//	This bit selects the hardware parity control (generation and detection). When the parity
	//	control is enabled, the computed parity is inserted at the MSB position (9th bit if M=1; 8th bit
	//	if M=0) and parity is checked on the received data. This bit is set and cleared by software.
	//	Once it is set, PCE is active after the current byte (in reception and in transmission).
	//	0: Parity control disabled
	//	1: Parity control enabled

	//	Bit 9 PS: Parity selection
	//	This bit selects the odd or even parity when the parity generation/detection is enabled (PCE
	//	bit set). It is set and cleared by software. The parity will be selected after the current byte.
	//	0: Even parity
	//	1: Odd parity
	USARTx->CR1 |=UART_cnfg->parity ;

	//STEP 6
	//	Bits 31:16 Reserved, forced by hardware to 0.
	//	Bits 15:4 DIV_Mantissa[11:0]: mantissa of USARTDIV
	//	These 12 bits define the mantissa of the USART Divider (USARTDIV)
	//	Bits 3:0 DIV_Fraction[3:0]: fraction of USARTDIV
	//	These 4 bits define the fraction of the USART Divider (USARTDIV)
	uint16_t DIV_Mantissa =(UART_cnfg->Clock) /((UART_cnfg->BaudRate)*16);
	uint16_t DIV_Fraction =((((UART_cnfg->Clock) /(((UART_cnfg->BaudRate)*16)/100)) - DIV_Mantissa*100)*16)/100;
	USARTx->BRR =( (DIV_Mantissa<<4) | ((DIV_Fraction)& 0xf) );

	//STEP 7
	//Set HWFlowCtl
	USARTx->CR3 &=~(0b11 << 8);
	USARTx->CR3 |=((UART_cnfg->HWFlowCtl) << 8);

	//Step 8 Enable /Disable USART interrupt
	if(UART_cnfg->IRQ_Enable != UART_IRQ_Enable_NONE)
	{
		USARTx->CR1 |=UART_cnfg->IRQ_Enable ;
		//Enable NVIC
		if(USARTx == USART1)
		{
			MCAL_NVIC_ENABLE(NVIC_USART1, NVIC_ENABLE);
		}
		else if(USARTx == USART2)
		{
			MCAL_NVIC_ENABLE(NVIC_USART2, NVIC_ENABLE);
		}
		else if(USARTx == USART3)
		{
			MCAL_NVIC_ENABLE(NVIC_USART3, NVIC_ENABLE);
		}

		GP_reseve_Callback[USART_Index] = Global_UART_config[USART_Index]->P_IRQ_CallBack;

	}

}

//==========================================================================================
void MCAL_UART_DeInit(USART_TypeDef* USARTx)
{
	if(USARTx == USART1)
	{
		MCAL_RCC_Peripherals_enable(APB2, RCC_USART1, RCC_Disable);
		MCAL_RCC_Peripherals_Reset(APB2, RCC_USART1, RCC_Reset);
	}
	else if(USARTx == USART2)
	{
		MCAL_RCC_Peripherals_enable(APB1, RCC_USART2, RCC_Disable);
		MCAL_RCC_Peripherals_Reset(APB1, RCC_USART2, RCC_Reset);
	}
	else if (USARTx == USART2)
	{
		MCAL_RCC_Peripherals_enable(APB1, RCC_USART3, RCC_Disable);
		MCAL_RCC_Peripherals_Reset(APB1, RCC_USART3, RCC_Reset);
	}

}

//==========================================================================================
void MCAL_UART_SendData(USART_TypeDef* USARTx , uint16_t* pTxBuffer , USART_Polling_mechanism_t PollingEn)
{
	if (PollingEn == enable)
	{
		while (! (USARTx->SR & 1<<7));
	}

	if(Global_UART_config[USART_Index]->payload_length == word_length_9)
	{
		USARTx->DR = (*pTxBuffer & (uint16_t)0x01FF);
	}
	else
	{
		USARTx->DR = (*pTxBuffer & (uint16_t)0xFF);
	}

}

//==========================================================================================
void MCAL_UART_ReceiveData(USART_TypeDef* USARTx , uint16_t* pRxBuffer , USART_Polling_mechanism_t PollingEn)
{
	//wait until RXNE flag is set in the SR
	if(PollingEn ==enable)
	{
		while(! (USARTx->SR & 1 << 5));
	}

	//check the USART_Wordlength item for 9BIT or 8 BIT in the frame
	if(Global_UART_config[USART_Index]->payload_length == word_length_9)
	{
		if(Global_UART_config[USART_Index]->parity == parity_none)
		{
			*((uint16_t*)pRxBuffer) = USARTx->DR;
		}
		else
		{
			*((uint16_t*)pRxBuffer) = (USARTx->DR & (uint8_t)0xFF);
		}
	}
	else
	{
		if(Global_UART_config[USART_Index]->parity == parity_none)
		{
			*((uint16_t*)pRxBuffer) = (USARTx->DR & (uint8_t)0xFF);
		}
		else
		{
			*((uint16_t*)pRxBuffer) = (USARTx->DR & (uint8_t)0x7F);
		}
	}

}

void USART1_IRQHandler()
{
	if((((USART1->SR >>7) & 1 ) == 1 ) && (((USART1->CR1 >>7)& 1 ) == 1))
	{

		if((*arr_data[0])!=0){
			MCAL_UART_SendData(USART1, &arr_data[0], enable);
			++arr_data[0];
		}
		else {
			//Bit 7 TXEIE: TXE interrupt enable
			//This bit is set and cleared by software.
			//0: Interrupt is inhibited
			//1: A USART interrupt is generated whenever TXE=1 in the USART_SR register
			USART1->CR1 &=~UART_IRQ_Enable_TXE;
			//arr[0]->USART_SR |=(1<<6);
		}
	}

	//===========================================================================

	if(  (( (USART1->SR>>5) & 1) ==1)&&(( (USART1->CR1 >>5) &1)==1) ){
		//		Bit 5 RXNE: Read data register not empty
		//		This bit is set by hardware when the content of the RDR shift register has been transferred to
		//		the USART_DR register. An interrupt is generated if RXNEIE=1 in the USART_CR1 register.
		//		It is cleared by a read to the USART_DR register. The RXNE flag can also be cleared by
		//		writing a zero to it. This clearing sequence is recommended only for multibuffer
		//		communication.
		//		0: Data is not received
		//		1: Received data is ready to be read



		//	Bit 12 M: Word length
		//	This bit determines the word length. It is set or cleared by software.
		if(((USART1->CR1>>12)&1) ==1){
			//	1: 1 Start bit, 9 Data bits, n Stop bit

			if(((USART1->CR1>>10)&1) ==1){
				//Bit 10 PCE: Parity control enable
				// 1: Parity control enabled
				arr_data_RES[0] =(USART1->DR & (uint16_t)0xff);
			}
			else{
				//0: Parity control disabled
				arr_data_RES[0]=USART1->DR;
			}
		}
		else{
			//	0: 1 Start bit, 8 Data bits, n Stop bit

			if(((USART1->CR1>>10)&1) ==1){
				//Bit 10 PCE: Parity control enable
				// 1: Parity control enabled
				arr_data_RES[0] =(USART1->DR & (uint16_t)0x7f);
			}
			else{
				//0: Parity control disabled
				arr_data_RES[0] =(USART1->DR & (uint16_t)0xff);
			}
		}
		//pinwrite(GPIOB, pin1,HIGH);

		GP_reseve_Callback[0]();

		for(long i=0;i<(20000);i++);
		//pinwrite(GPIOB, pin1,LOW);
		USART1->SR &=~(1<<5);//clear
		//arr[0]->USART_CR1 &=~Received_data_ready_to_be_read;
		if(((USART1->SR>>3)&1)|1){
			volatile uint16_t d=USART1->DR;
			USART1->SR &=~(1<<5);
		}
	}

	if(((USART1->SR >> 3)& 1)|1)
	{
		volatile uint16_t w =USART1->DR;
		USART1->SR &=~(1<<5);
	}
}


void USART2_IRQHandler()
{
	if((((USART2->SR >>7) & 1 ) == 1 ) && (((USART2->CR1 >>7)& 1 ) == 1))
	{

		if((*arr_data[0])!=0){
			MCAL_UART_SendData(USART2, &arr_data[0], enable);
			++arr_data[0];
		}
		else {
			//Bit 7 TXEIE: TXE interrupt enable
			//This bit is set and cleared by software.
			//0: Interrupt is inhibited
			//1: A USART interrupt is generated whenever TXE=1 in the USART_SR register
			USART2->CR1 &=~UART_IRQ_Enable_TXE;
			//arr[0]->USART_SR |=(1<<6);
		}
	}

	//===========================================================================

	if(  (( (USART2->SR>>5) & 1) ==1)&&(( (USART2->CR1 >>5) &1)==1) ){
		//		Bit 5 RXNE: Read data register not empty
		//		This bit is set by hardware when the content of the RDR shift register has been transferred to
		//		the USART_DR register. An interrupt is generated if RXNEIE=1 in the USART_CR1 register.
		//		It is cleared by a read to the USART_DR register. The RXNE flag can also be cleared by
		//		writing a zero to it. This clearing sequence is recommended only for multibuffer
		//		communication.
		//		0: Data is not received
		//		1: Received data is ready to be read



		//	Bit 12 M: Word length
		//	This bit determines the word length. It is set or cleared by software.
		if(((USART2->CR1>>12)&1) ==1){
			//	1: 1 Start bit, 9 Data bits, n Stop bit

			if(((USART2->CR1>>10)&1) ==1){
				//Bit 10 PCE: Parity control enable
				// 1: Parity control enabled
				arr_data_RES[0] =(USART2->DR & (uint16_t)0xff);
			}
			else{
				//0: Parity control disabled
				arr_data_RES[0]=USART2->DR;
			}
		}
		else{
			//	0: 1 Start bit, 8 Data bits, n Stop bit

			if(((USART2->CR1>>10)&1) ==1){
				//Bit 10 PCE: Parity control enable
				// 1: Parity control enabled
				arr_data_RES[0] =(USART2->DR & (uint16_t)0x7f);
			}
			else{
				//0: Parity control disabled
				arr_data_RES[0] =(USART2->DR & (uint16_t)0xff);
			}
		}
		//pinwrite(GPIOB, pin1,HIGH);

		GP_reseve_Callback[0]();

		for(long i=0;i<(20000);i++);
		//pinwrite(GPIOB, pin1,LOW);
		USART2->SR &=~(1<<5);//clear
		//arr[0]->USART_CR1 &=~Received_data_ready_to_be_read;
		if(((USART2->SR>>3)&1)|1){
			volatile uint16_t d=USART2->DR;
			USART2->SR &=~(1<<5);
		}
	}

	if(((USART2->SR >> 3)& 1)|1)
	{
		volatile uint16_t w =USART2->DR;
		USART2->SR &=~(1<<5);
	}
}

void USART3_IRQHandler()
{
	if((((USART3->SR >>7) & 1 ) == 1 ) && (((USART3->CR1 >>7)& 1 ) == 1))
	{

		if((*arr_data[0])!=0){
			MCAL_UART_SendData(USART3, &arr_data[0], enable);
			++arr_data[0];
		}
		else {
			//Bit 7 TXEIE: TXE interrupt enable
			//This bit is set and cleared by software.
			//0: Interrupt is inhibited
			//1: A USART interrupt is generated whenever TXE=1 in the USART_SR register
			USART3->CR1 &=~UART_IRQ_Enable_TXE;
			//arr[0]->USART_SR |=(1<<6);
		}
	}

	//===========================================================================

	if(  (( (USART3->SR>>5) & 1) ==1)&&(( (USART3->CR1 >>5) &1)==1) ){
		//		Bit 5 RXNE: Read data register not empty
		//		This bit is set by hardware when the content of the RDR shift register has been transferred to
		//		the USART_DR register. An interrupt is generated if RXNEIE=1 in the USART_CR1 register.
		//		It is cleared by a read to the USART_DR register. The RXNE flag can also be cleared by
		//		writing a zero to it. This clearing sequence is recommended only for multibuffer
		//		communication.
		//		0: Data is not received
		//		1: Received data is ready to be read



		//	Bit 12 M: Word length
		//	This bit determines the word length. It is set or cleared by software.
		if(((USART3->CR1>>12)&1) ==1){
			//	1: 1 Start bit, 9 Data bits, n Stop bit

			if(((USART3->CR1>>10)&1) ==1){
				//Bit 10 PCE: Parity control enable
				// 1: Parity control enabled
				arr_data_RES[0] =(USART3->DR & (uint16_t)0xff);
			}
			else{
				//0: Parity control disabled
				arr_data_RES[0]=USART3->DR;
			}
		}
		else{
			//	0: 1 Start bit, 8 Data bits, n Stop bit

			if(((USART3->CR1>>10)&1) ==1){
				//Bit 10 PCE: Parity control enable
				// 1: Parity control enabled
				arr_data_RES[0] =(USART3->DR & (uint16_t)0x7f);
			}
			else{
				//0: Parity control disabled
				arr_data_RES[0] =(USART3->DR & (uint16_t)0xff);
			}
		}
		//pinwrite(GPIOB, pin1,HIGH);

		GP_reseve_Callback[0]();

		for(long i=0;i<(20000);i++);
		//pinwrite(GPIOB, pin1,LOW);
		USART3->SR &=~(1<<5);//clear
		//arr[0]->USART_CR1 &=~Received_data_ready_to_be_read;
		if(((USART3->SR>>3)&1)|1){
			volatile uint16_t d=USART3->DR;
			USART3->SR &=~(1<<5);
		}
	}

	if(((USART3->SR >> 3)& 1)|1)
	{
		volatile uint16_t w =USART3->DR;
		USART3->SR &=~(1<<5);
	}
}
