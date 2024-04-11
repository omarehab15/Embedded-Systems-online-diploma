/*
 * UTILS.h
 *
 * Created: 3/23/2024 2:43:37 PM
 *  Author: omar
 */ 


#ifndef UTILS_H_
#define UTILS_H_
#define SET_BIT(reg , bit)		(reg |= (1<< bit))
#define CLEAR_BIT(reg , bit)	(reg &=~ (1<< bit))
#define READ_BIT(reg , bit)		((reg >> bit)&1)
#define Toggle_BIT(reg , bit)	(reg ^= (1<< bit))


#endif /* UTILS_H_ */