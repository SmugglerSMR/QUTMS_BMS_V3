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

#define MAX14920_PIN_SCK	PINB7
#define MAX14920_PIN_MOSI	PINB1
#define MAX14920_PIN_MISO	PINB0
#define MAX14920_PIN_SS		PIND3		//***

#define MAX14920_PORT_CS	PORTC
#define MAX14920_PIN_CS		PINC3		//***

#define MAX14920_PORT_EN	PORTD
#define MAX14920_PIN_EN		PIND7		//***

#define MAX14920_PORT_SAMPL	PORTC
#define MAX14920_PIN_SAMPL	PINC6		//***

#define MAX14920_SMPLB_bit	3

#define MAX14920_CELL_NUMBER	12

#define SPI_SCK_PIN	7 //PB7
#define SPI_MOSI_PIN	1 //PB1
#define SPI_MISO_PIN	0 //PB0
 

static float OveralVoltage = 0;
static uint16_t AverageCellVoltage = 0;
static uint16_t MaxCellVoltage = 0;
static uint16_t MinCellVoltage = 0;

void MAX14920_Init_Registers(void);	
void MAX14920_Clear_SPI_messages(void);
void MAX14920_reg_write(void);
void MAX14920_Enable(void);
void MAX14920_OffsetCallibration(void);
void MAX14920_EnableHoldPhase(bool enable);

uint16_t MAX14920_ReadData(void);
//uint16_t MAX14920_ReadCellVoltage(int cellN);
float MAX14920_ReadCellVoltage(int cellN);
void MAX14920_ReadAllCellsVoltage(float CellVoltages[]);
void MAX14920_EnableLoadBalancer(bool enable);
void MAX14920_PerformDiagnosticsFirst(void);
void MAX14920_PerformDiagnosticsSecond(void);

extern uint16_t DecToBin(float nn);
extern void Toggle_LED(int id, int delay, int times);

#endif /* MAX14920_H_ */