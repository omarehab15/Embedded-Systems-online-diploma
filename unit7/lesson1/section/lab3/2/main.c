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
	static unsigned char seven_segment[] = {0x3F, 0x06 , 0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
	CLEAR_BIT(DDRD, 0);
	SET_BIT(DDRD, 4);
	CLEAR_BIT(PORTD, 0 );
    while (1) 
    {
		if((READ_BIT(PIND, 0))==1)
		{
			SET_BIT(PORTD, 4 );
		}
		else
		{
			CLEAR_BIT(PORTD, 4 );
		}
	}
}

