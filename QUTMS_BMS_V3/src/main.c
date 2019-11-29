/**
 * \file
 *
 * \brief A third version of Battery Management System code using MAX chips
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#define F_CPU 16000000UL

#include <asf.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>

#include "macros.h"
#include "SPI.h"
#include "ADC.h"
#include "MAX14920.h"
#include "HC595PW.h"
#include "MCP2517FD.h"

void IO_init(void);
void Toggle_LED(int id, int delay, int times);
uint16_t DecToBin(float nn);

//static uint16_t CellVoltages[10];
//extern uint16_t OveralVoltage;
//extern uint16_t Max_Resistance;
//extern uint16_t AverageCellVoltage;


void IO_init(void) {
	// Initialize LEDs
	DDRB = 0b00011000;	// LED-5 LED-4
	DDRC = 0b00000001;	// LED-3 
	DDRD = 0b00000011;	// LED 7 6
}

/*
	Function used to toggle LED with a delay.
	int id:	ID as appear on a board
	int delay: Time in ms
*/
void Toggle_LED(int id, int delay, int times) {
	for(int i = 0; i < times; i++) {		
		switch(id) {		
			case 5:		// red
				PORTB ^= 0b00010000;
				for (int i = 0; i < delay/2; i++)	{
					_delay_ms(1);
				}
				PORTB ^= 0b00010000;
				break;			
			case 4:		// blue
				PORTB ^= 0b00001000;
				for (int i = 0; i < delay/2; i++)	{
					_delay_ms(1);
				}
				PORTB ^= 0b00001000;
				break;
			case 3:		// blue
				PORTC ^= 0b00000001;
				for (int i = 0; i < delay/2; i++)	{
					_delay_ms(1);
				}
				PORTC ^= 0b00000001;
				break;		
			case 7:		// white
				PORTD ^= 0b00000010;
				for (int i = 0; i < delay/2; i++)	{
					_delay_ms(1);
				}
				PORTD ^= 0b00000010;
				break;
			case 6:		// red
				PORTD ^= 0b00000001;
				for (int i = 0; i < delay/2; i++)	{
					_delay_ms(1);
				}
				PORTD ^= 0b00000001;
				break;
		}
		for (int i = 0; i < delay/2; i++)	{
			_delay_ms(1);
		}
	}
}

uint16_t DecToBin(float nn) {
	int a[8];
	uint8_t byte = 0;
	int n = (int) nn;	
	for(int i=0;n>0;i++){
		a[i]=n%2;
		n=n/2;
	}
	for(int i=0;n>8;i++){
		if(a[i] == 1) {
			byte |= 1;
			byte <<=1;
		} else {
			byte |= 0;
			byte <<=1;
		}
	}
	return byte;
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	// Initialize ATmega64M1 micro controller
	//board_init();
	IO_init();
	SPI_init();
	ADC_init();
	MAX14920_Init_Registers();
	HC595PW_Init_Registers();
	MCP2517FD_Init_Registers();
	
	// Initialize MAX14920 micro controller
	MAX14920_Clear_SPI_messages();
	MAX14920_Enable();
	// Perform Diagnostics
	MAX14920_PerformDiagnosticsFirst();
	//MAX14920_PerformDiagnosticsSecond();
	
	// CAN MEssage
	MCP2517_init();
	BMS_BOARD_DATA[0] = 1;	//Board functionality	
	
	// Loop forever for checks	
	uint8_t cycle = 0;
	uint8_t data[8] = {0};
	uint32_t receiveID;
	uint8_t numDataBytes;
	
	while(1) {
		Toggle_LED(7, 1000,1);
		
		//////////////////////////////////////////
		// MAX14920  - Cell Manipulations
		// Do not read voltage during Cell balancing at all for now
		//if(~MAX14920_SPI_message.spiBalanceC01_C08 && ~MAX14920_SPI_message.spiBalanceC01_C08) {
			MAX14920_ReadAllCellsVoltage();
			_delay_ms(50);
			OveralVoltage = MAX14920_ReadCellVoltage(0);
		//}
		
		
		// Report fault on any of the cells
		//if(~MAX14920_SPI_output.spiCellStatusC01_C08 ||
		   //~MAX14920_SPI_output.spiCellStatusC09_C16) {
			//Toggle_LED(5,500,1);
		//}
		
		// Toggle balancer
		// TODO: Recheck values before playing with balancer
		// TODO: Make sure that values for threshold is accurate first
		// This one only for charge.
		//MAX14920_EnableLoadBalancer(true);
		
		//////////////////////////////////////////
		// 74HC595PW - Start Temperature readings
		// TODO: Record temperature for CAN
		HC595PW_CD74HCT_send_read();
		
		//////////////////////////////////////////
		// MCP2517FD - CANBUS		
		BMS_BOARD_DATA[1] = OveralVoltage;	//Board functionality
		BMS_BOARD_DATA[2] = AverageCellVoltage;	//Board functionality
		BMS_BOARD_DATA[3] = Max_Resistance;	//Board functionality
		BMS_BOARD_DATA[4] = cycle;	//Board functionality
		
		MCP2517_recieveMessage(&receiveID, &numDataBytes, data);		
		if(receiveID == CAN_ID_BMS >> 18) {
			PORTD ^= 0b00000001;
			MCP2517_transmitMessage(CAN_ID_AMS, 5, BMS_BOARD_DATA);
			_delay_ms(50);
			PORTD ^= 0b00000001;
		}
		
		cycle++;
		if(cycle >=200)
			cycle = 0;
		
	}
}
