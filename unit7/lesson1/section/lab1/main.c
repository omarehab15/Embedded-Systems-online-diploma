/*
 * AVR_Drivers.c
 *
 * Created: 3/23/2024 2:00:07 PM
 * Author : omar
 */ 


#include "ATMEGA32.h"
#include "util/delay.h"
#define F_CPU	8000000UL

int main(void)
{
	DDRA = 0xFF;
    /* Replace with your application code */
    while (1) 
    {
		PORTA |=(1 << 0)  ;
		_delay_ms(1500);
		PORTA |=(1 << 1)  ;
		_delay_ms(1500);
		PORTA |=(1 << 2)  ;
		_delay_ms(1500);
		PORTA |=(1 << 3)  ;
		_delay_ms(1500);
		PORTA |=(1 << 4)  ;
		_delay_ms(1500);
		PORTA |=(1 << 5)  ;
		_delay_ms(1500);
		PORTA |=(1 << 6)  ;
		_delay_ms(1500);
		PORTA |=(1 << 7)  ;
		_delay_ms(1500);
		PORTA &=~(1 << 7)  ;
		_delay_ms(1500);
		PORTA &=~(1 << 6)  ;
		_delay_ms(1500);
		PORTA &=~(1 << 5)  ;
		_delay_ms(1500);
		PORTA &=~(1 << 4)  ;
		_delay_ms(1500);
		PORTA &=~(1 << 3)  ;
		_delay_ms(1500);
		PORTA &=~(1 << 2)  ;
		_delay_ms(1500);
		PORTA &=~(1 << 1)  ;
		_delay_ms(1500);
		PORTA &=~(1 << 0)  ;
		_delay_ms(1500);
		
		
    }
}

