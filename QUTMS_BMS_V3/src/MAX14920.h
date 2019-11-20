/*
 * MAX14920.h
 *
 * Created: 19/11/2019 3:48:32 PM
 *  Author: sadykov
 */ 


#ifndef MAX14920_H_
#define MAX14920_H_

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#define MAX14920_PORT_CS	PORTC
#define MAX14920_PIN_CS		PINC3		//***

#define MAX14920_PORT_EN	PORTD
#define MAX14920_PIN_EN		PIND7		//***

#define MAX14920_PORT_SMPLB	PORTC
#define MAX14920_PIN_SMPLB	PINC6		//***

void MAX14920_reg_write(uint8_t CB1_CB8, uint8_t CB9_CB16, uint8_t ECS);
void MAX14920_Enable(void);
void MAX14920_OffsetCallibration();
void MAX14920_ReadData(void);
void MAX14920_ReadCellVoltage(int cellN);
void MAX14920_ReadAllCellsVoltage();

extern int * DecToBin(double nn);
extern void Toggle_LED(int id, int delay, int times);

#endif /* MAX14920_H_ */