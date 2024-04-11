/*
 * AVR_Drivers.c
 *
 * Created: 3/23/2024 2:00:07 PM
 * Author : omar
 */ 


#include "ATMEGA32.h"
#include "utils.h"
#include "util/delay.h"
#define F_CPU	8000000UL



int main(void)
{
	unsigned char c = 0 , f = 0 ;
	DDRA = 0xFF;
	CLEAR_BIT(DDRC , 0);
    SET_BIT(PORTC , 0);
    while (1) 
    {
		if((READ_BIT(PINC , 0)) == 0)
		{
			SET_BIT(PORTA , c) ;
			_delay_ms(5000);
			c++;
		}
		
		
		
    }
}

