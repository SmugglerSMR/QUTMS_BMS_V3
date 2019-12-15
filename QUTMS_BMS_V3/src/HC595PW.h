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

//NCP18XH103F03RB - Related variables
#define RES_VALUE 10000
#define THERMISTORNOMINAL 10000
//+1.7K is the offset
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3380
#define SAMPLING 15

static uint16_t Max_Resistance = 10000;
static uint16_t Min_Resistance = 10000;
static uint16_t Average_Resistance = 10000;

static uint16_t CellResistance_One[32] = {0};
	static uint16_t CellResistance_Two[32] = {0};

void HC595PW_Init_Registers(void);
void HC595Pulse(void);
void HC595Latch(void);
void HC595PW_reg_write(uint8_t data);
float HC595_CalcTemp(uint16_t resistance);
void HC595PW_CD74HCT_send_read(void);

extern uint16_t DecToBin(float nn);
extern void Toggle_LED(int id, int delay, int times);

#endif /* 74HC595PW_H_ */