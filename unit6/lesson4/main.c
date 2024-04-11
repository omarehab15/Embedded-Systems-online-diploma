/*
 * interrupt_lab.c
 *
 * Created: 12/4/2023 4:50:25 PM
 * Author : omar
 */ 

#include <avr/io.h>
#include <util/delay.h> 
#include <avr/interrupt.h>

#define F_CPU	8000000UL

#define SET_BIT(ADRESS , BIT)		(ADRESS |= (1 << BIT)) 
#define RESET_BIT(ADRESS , BIT)		(ADRESS &=~ (1 << BIT)) 
#define TOGGLE_BIT(ADRESS , BIT)	(ADRESS ^= (1 << BIT)) 
#define READ_BIT(ADRESS , BIT)		((ADRESS & (1 << BIT)) >> BIT) 

#define IO_BASE		0x20
#define IO_PORTD	*(volatile unsigned char*)(IO_BASE + 0x12)
#define IO_DDRD		*(volatile unsigned char*)(IO_BASE + 0x11)
#define INT_MCUCR	*(volatile unsigned char*)(IO_BASE + 0x35)
#define INT_GICR 	*(volatile unsigned char*)(IO_BASE + 0x3B)
#define INT_MCUCSR	*(volatile unsigned char*)(IO_BASE + 0x34)

int main(void)
{
   // Set pins (5 , 6 , 7) as a output in port D 
   SET_BIT(IO_DDRD , 5) ;
   SET_BIT(IO_DDRD , 6) ;
   SET_BIT(IO_DDRD , 7) ;
   //set int0 logical 
   SET_BIT( INT_MCUCR , 0) ;
   RESET_BIT(INT_MCUCR , 1) ;
   //set int1 rising edge 
   SET_BIT(INT_MCUCR , 2) ;
   SET_BIT(INT_MCUCR , 3) ;
   //set int2 falling edge
   RESET_BIT(INT_MCUCSR , 6) ;
   //enable int 0 , 1 & 2  
    SET_BIT(INT_GICR , 5) ;
	SET_BIT(INT_GICR , 6) ;
	SET_BIT(INT_GICR , 7) ;
	//enable global interrupt
	sei();
	
	while(1)
	{
		 RESET_BIT(IO_PORTD , 5) ;
		 RESET_BIT(IO_PORTD , 6) ;
		 RESET_BIT(IO_PORTD , 7) ;
	}
	
   return 0;

}

 ISR (INT0_vect)
 {
	 SET_BIT(IO_PORTD , 5) ;
	 _delay_ms (1000);
 }
 
 ISR (INT1_vect)
 {
	 SET_BIT(IO_PORTD , 6) ;
	 _delay_ms (1000);
 }

 ISR (INT2_vect)
 {
	 SET_BIT(IO_PORTD , 7) ;
	 _delay_ms (1000);
 }
 