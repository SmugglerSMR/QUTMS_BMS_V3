/*
 * _74HC595PW.h
 *
 * Created: 22/11/2019 8:44:29 AM
 *  Author: sadykov
 */ 


#ifndef HC595PW_H_
#define HC595PW_H_

#include <avr/io.h>
#include <util/delay.h>

#define HC595PW_PORT_MR	PORTB
#define HC595PW_PIN_MR	PINB6

#define HC595PW_PORT_DC	PORTC
#define HC595PW_PIN_DC	PINC2

#define HC595PW_PORT_ST	PORTB
#define HC595PW_PIN_ST	PINB2

#define HC595PW_PORT_SH	PORTB
#define HC595PW_PIN_SH	PINB7

static volatile double SensorTemp[64] = {0.0};
	
#endif /* 74HC595PW_H_ */