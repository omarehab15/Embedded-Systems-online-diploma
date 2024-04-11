/*
 * ATMEGA32.h
 *
 * Created: 3/23/2024 2:03:54 PM
 *  Author: omar
 */ 


#ifndef ATMEGA32_H_
#define ATMEGA32_H_


#define	PORTA	*((volatile unsigned char *) 0x3B)
#define	DDRA	*((volatile unsigned char *) 0x3A)
#define	PINA	*((volatile unsigned char *) 0x39)


#define	PORTB	*((volatile unsigned char *) 0x38)
#define	DDRB	*((volatile unsigned char *) 0x37)
#define	PINB	*((volatile unsigned char *) 0x36)


#define	PORTC	*((volatile unsigned char *) 0x35)
#define	DDRC	*((volatile unsigned char *) 0x34)
#define	PINC	*((volatile unsigned char *) 0x33)


#define	PORTD	*((volatile unsigned char *) 0x32)
#define	DDRD	*((volatile unsigned char *) 0x31)
#define	PIND	*((volatile unsigned char *) 0x30)


#endif /* ATMEGA32_H_ */