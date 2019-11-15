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

void IO_init() {
	// Initialise LEDs
	DDRB = 0b00011000;	// LED 5 4
	DDRC = 0b00001001;	// LED 3
	DDRD = 0b00000011;	// LED 7 6
	
	PORTC |= (1<<PINC3); // Set SS as output
}

/*
	Function used to toggle LED with a delay.
	int id:	ID as appear on a board
	int delay: Time in ms
*/
void Toggle_LED(int id, int delay) {
	switch(id) {		
		case 5:		// red
			PORTB ^= 0b00010000;
			break;			
		case 4:		// blue
			PORTB ^= 0b00001000;
			break;
		case 3:		// blue
			PORTC ^= 0b00000001;
			break;		
		case 7:		// white
			PORTD ^= 0b00000010;
			break;
		case 6:		// red
			PORTD ^= 0b00000001;
			break;
	}
	for (int i = 0; i < delay; i++)	{
		_delay_ms(1);
	}
}

/*
	Simply speaking - Morze code.
	Use it to chech if bits in register as expected.
	TOSO: Rewirete it to fit registers.
*/
void Morzanka(int code[]) {
	for (int i = 0; i < 8; i++) {
		if(code[i] == 0) Toggle_LED(4,500);
		else Toggle_LED(4,2000);
		Toggle_LED(4,200);
	}
	Toggle_LED(4,5000);
}

#define MAX14920_PORT_CS	PORTC
#define MAX14920_PIN_CS		PINC3		//***

void MAX14920_reg_write(uint8_t balanc_one, uint8_t balanc_two, uint8_t cells) {
	uint8_t output;
	MAX14920_PORT_CS &= ~(1<<MAX14920_PIN_CS); // unset to start transmission
	SPI_send_byte(balanc_one);
	SPI_send_byte(balanc_two);
	output = SPI_send_byte(cells);
	MAX14920_PORT_CS |= 1<<MAX14920_PIN_CS; // Set back.
}
int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	board_init();
	IO_init();
	SPI_init();
	/* Insert application code here, after the board has been initialized. */
	
	MAX14920_reg_write(0x00,0x00,0x84)
	while(1) {
		Toggle_LED(7, 1500);
	}
}
