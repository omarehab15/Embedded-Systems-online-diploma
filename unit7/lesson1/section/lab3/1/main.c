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
	DDRC = 0xFF;
    while (1) 
    {
		for(int i = 0 ; i < 10 ; i++)
		{
			PORTC = seven_segment[i] ;
			_delay_ms(5500);
		}
		
		
    }
}

