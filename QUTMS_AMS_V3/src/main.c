/**
 * \file
 *
 * \brief Empty user application template
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

#include "SPI.h"

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	DDRB = 0b10110000;	//SS-output, MOSI-out, CLK-out
	DDRA = 0b00000010;	//
		
	//board_init();
	
	SPI_init();
	PORTA |= (1<<PINA1); // Disable CS
	//PORTB |= (1<<PINB4);
	
	//PORTB &= ~(1<<PINB5);
	_delay_ms(300);
	/* Insert application code here, after the board has been initialized. */
	while(1) {
		SPI_send_byte(0b00011000);
		//PORTB ^= 0b00100000;
		_delay_ms(500);
	}
}
