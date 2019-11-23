/*
 * _74HC595PW.h
 *
 * Created: 22/11/2019 8:44:29 AM
 *  Author: sadykov
 */ 


#ifndef HC595PW_H_
#define HC595PW_H_

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#define HC595PW_PORT_MR	PORTB
#define HC595PW_PIN_MR	PINB6

#define HC595PW_PORT_DS	PORTC
#define HC595PW_PIN_DS	PINC2

#define HC595PW_PORT_ST	PORTB
#define HC595PW_PIN_ST	PINB2

#define HC595PW_PORT_SH	PORTB
#define HC595PW_PIN_SH	PINB7

static volatile double SensorTemp[64] = {0.0};

void HC595PW_Init_Registers(void);
void HC595Pulse(void);
void HC595Latch(void);
void HC595PW_reg_write(uint8_t data);
void HC595PW_CD74HCT_send_read(void);
#endif /* 74HC595PW_H_ */