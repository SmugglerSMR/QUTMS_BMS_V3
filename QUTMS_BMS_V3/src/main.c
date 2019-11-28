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
uint8_t DecToBin(double nn);

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

///*
	//Simply speaking - Morze code.
	//Use it to chech if bits in register as expected.
	//TOSO: Rewirete it to fit registers.
//*/
//void Morzanka(int code[]) {
	//for (int i = 0; i < 8; i++) {
		//if(code[i] == 0) Toggle_LED(4,500);
		//else Toggle_LED(4,2000);
		//Toggle_LED(4,200);
	//}
	//Toggle_LED(4,5000);
//}

uint8_t DecToBin(double nn) {
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
	//MCP2517FD_Init_Registers();
	
	// Initialize MAX14920 micro controller
	MAX14920_Clear_SPI_messages();
	MAX14920_Enable();
	MAX14920_EnableHoldPhase(false);
		
	// Loop forever for checks
	uint16_t overallVoltage = 0;
	while(1) {
		Toggle_LED(7, 1000,1);
		//MAX14920_ReadAllCellsVoltage();
		_delay_ms(50);
		overallVoltage = MAX14920_ReadCellVoltage(2);
		//for(int i=0; i<10; i++)
			//SPI_send_byte((CellVoltages[i]));
		//SPI_send_byte((overallVoltage));
		
		// Report fault on any of the cells
		if(MAX14920_SPI_output.spiCellStatusC01_C08 ||
		   MAX14920_SPI_output.spiCellStatusC09_C16) {
			Toggle_LED(5,500,1);
		}
		
		// Toggle balancer
		// TODO: Recheck values before playing with balancer
		// TODO: Make sure that values for threshold is accurate first
		// This one only for charge.
		//MAX14920_EnableLoadBalancer(true);
		
		// Start Temperature readings
		HC595PW_CD74HCT_send_read();
		//for(int i=0; i<64;i++)
			//SPI_send_byte(adc_read(2) >> 2);
	}
}
