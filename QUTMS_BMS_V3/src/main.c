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



void IO_init(void);
void Toggle_LED(int id, int delay, int times);
int * DecToBin(double nn);

void IO_init(void) {
	// Initialise LEDs
	DDRB = 0b10011010;	// CLK-output LED 5 4 MOSI-output MISO-input
	DDRC = 0b01001001;	// SAMPL-low CS-high LED 3 
	DDRD = 0b10001011;	// EN-MAX SS-high LED 7 6
	
	//PORTC |= (1<<PINC3); 
	//PORTC |= (1<<PINC6); 
	
	// Set SS as high to disable transmission.
	WRITE_BIT(MAX14920_PORT_CS, MAX14920_PIN_CS, HIGH);
	// TODO: Try make MISO and MOSO low
	
	// Allow sampling through SPI messages.
	WRITE_BIT(MAX14920_PORT_SAMPL, MAX14920_PIN_SAMPL, HIGH);
	
	// Shutdown and reset SPI.
	WRITE_BIT(MAX14920_PORT_EN, MAX14920_PIN_EN, LOW);
	
	// TODO: Check LDO voltage. It should nit be higher than 5.25V
	//PORTB |= (1<<PINB1); //SET MOSi as output
	// TODO: COMLETE THOSE PARTS
	//PORTC |= (1<<PINC6) | (1<<PINC3); // Disable sampler and CS.
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

uint8_t * DecToBin(double nn) {
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
	
	// Initialize MAX14920 micro controller
	MAX14920_Clear_SPI_messages();
	MAX14920_Enable();
	MAX14920_EnableHoldPhase(false);
		
	// Loop to hold processor
	double overallVoltage = 0.0;
	while(1) {
		Toggle_LED(7, 1000,1);
		MAX14920_ReadAllCellsVoltage();
		_delay_ms(50);
		overallVoltage = MAX14920_ReadCellVoltage(0);
		SPI_send_byte(DecToBin(overallVoltage));
		
		// Report fault on any of the cells
		if(MAX14920_SPI_output.spiCellStatusC01_C08 ||
		   MAX14920_SPI_output.spiCellStatusC09_C16) {
			SPI_send_byte(0b1111001);
		}
		// Toggle balancer
		MAX14920_EnableLoadBalancer(true);
	}
}
