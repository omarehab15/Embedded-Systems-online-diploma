/*
 * AVR_Drivers.c
 *
 * Created: 3/23/2024 2:00:07 PM
 * Author : omar
 */ 


#include "ATMEGA32.h"
#include "utils.h"
#include "util/delay.h"
#include "LCD_Driver/LCD.h"


int main(void)
{
	static unsigned char seven_segment[] = {0x3F, 0x06 , 0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
	LCD_INIT();
	LCD_WRITE_STRING("OMAR EHAB 123 .....LEARN IN Depth");
    while (1) 
    {
		
	}
}

