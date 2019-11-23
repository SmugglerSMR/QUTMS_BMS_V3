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
#include <stdbool.h>
#include <stdlib.h>

#define MAX14920_PORT_CS	PORTC
#define MAX14920_PIN_CS		PINC3		//***

#define MAX14920_PORT_EN	PORTD
#define MAX14920_PIN_EN		PIND7		//***

#define MAX14920_PORT_SAMPL	PORTC
#define MAX14920_PIN_SAMPL	PINC6		//***

#define MAX14920_SMPLB_bit	3

#define MAX14920_CELL_NUMBER	12

static volatile double CellVoltages[10] = {0.0};
	
void MAX14920_Clear_SPI_messages(void);
void MAX14920_reg_write(void);
void MAX14920_Enable(void);
void MAX14920_OffsetCallibration(void);
void MAX14920_EnableHoldPhase(bool enable);

double MAX14920_ReadData(void);
double MAX14920_ReadCellVoltage(int cellN);
void MAX14920_ReadAllCellsVoltage(void);
void MAX14920_EnableLoadBalancer(bool enable);

extern uint8_t  DecToBin(double nn);
extern void Toggle_LED(int id, int delay, int times);

#endif /* MAX14920_H_ */