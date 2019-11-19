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

#include "SPI.h"
#include "macros.h"
#include "ADC.h"
#define MAX14920_PORT_CS	PORTC
#define MAX14920_PIN_CS		PINC3		//***

static const int cellTable[] = {
	0b0000, 0b1000, 0b0100, 0b1100,
	0b0010, 0b1010, 0b0110, 0b1110,
	0b0001, 0b1001, 0b0101, 0b1101
};

void IO_init() {
	// Initialise LEDs
	DDRB = 0b10011010;	// CLK-output LED 5 4 MOSI-output MISO-input
	DDRC = 0b01001001;	// SAMPL-low CS-high LED 3 
	DDRD = 0b00001011;	// SS-high LED 7 6
	
	//PORTC |= (1<<PINC3); 
	//PORTC |= (1<<PINC6); 
	SET_BIT(MAX14920_PORT_CS, MAX14920_PIN_CS); // Set SS as output high
	SET_BIT(PORTC, PINC6); // Set SAMPL high to track voltage at CV
		
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
				for (int i = 0; i < delay; i++)	{
					_delay_ms(1);
				}
				PORTB ^= 0b00010000;
				break;			
			case 4:		// blue
				PORTB ^= 0b00001000;
				for (int i = 0; i < delay; i++)	{
					_delay_ms(1);
				}
				PORTB ^= 0b00001000;
				break;
			case 3:		// blue
				PORTC ^= 0b00000001;
				for (int i = 0; i < delay; i++)	{
					_delay_ms(1);
				}
				PORTC ^= 0b00000001;
				break;		
			case 7:		// white
				PORTD ^= 0b00000010;
				for (int i = 0; i < delay; i++)	{
					_delay_ms(1);
				}
				PORTD ^= 0b00000010;
				break;
			case 6:		// red
				PORTD ^= 0b00000001;
				for (int i = 0; i < delay; i++)	{
					_delay_ms(1);
				}
				PORTD ^= 0b00000001;
				break;
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

int * DecToBin(double nn) {
	static int a[5];
	int n = (int) nn;	
	for(int i=0;n>0;i++){
		a[i]=n%2;
		n=n/2;
	}
	return a;
}

void MAX14920_reg_write(uint8_t CB1_CB8, uint8_t CB9_CB16, uint8_t ECS) {
	uint8_t output;
	
	//PORTC &= ~(1<<PINC6); // Sampl_disable 	
	//_delay_ms(50);
	//
	//PORTC &= ~(1<<PINC6); // Sampl_disable 			
	CLEAR_BIT(MAX14920_PORT_CS, MAX14920_PIN_CS); // unset to start transmission
	SPI_send_byte(CB1_CB8);
	SPI_send_byte(CB9_CB16);	
	output = SPI_send_byte(ECS);
	MAX14920_PORT_CS |= 1<<MAX14920_PIN_CS; // Set back.	
	SET_BIT(MAX14920_PORT_CS, MAX14920_PIN_CS);
	_delay_ms(10);
}

void MAX14920_ReadData() {
	PORTC &= ~(1<<PINC6); // Set SAMPL low to make ready
	// ADC output is next thing to do
	// Try get response or similar
	int bit_test = 0;
	
	//ADC0_BASE
	// TEST LINE for blicking
	//output = 0b0001000;
	
	// Send dummy value
	MAX14920_reg_write(0x00,0x00,0b00000100);
	//Getting ADC value		
	uint16_t ADC_v = adc_read(6);
	
	PORTC |= (1<<PINC6); // Set SAMPL high to track voltage at CV
	
	double res_voltage = ADC_v;
	//double res_voltage = 600/(double)ADC_v;
	int *p = DecToBin(res_voltage);
	//for(int i = 0; i < 5;i++) if(p[i]==1) Toggle_LED(3+i,500);
	
	//if(res_voltage<25) Toggle_LED(6,1);
	//else Toggle_LED(7,1);
	////
	//while (bit_test < 8) {
	//if (output & 0x01) {
	//Toggle_LED(4,2000);
	//}
	//else {
	//Toggle_LED(5,500);
	//}
	//
	//bit_test++;
	//output = output >> 1;
	//}	
}
int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	//board_init();
	IO_init();
	SPI_init();
	ADC_init();
	/* Insert application code here, after the board has been initialized. */
	//_delay_ms(3000);
	
	// Probing CEL1
	Toggle_LED(7, 150, 4);
	_delay_ms(500);
	MAX14920_reg_write(0b00000000,0b00000000,0b00000100);
	_delay_ms(50); // Wait for voltage to be shifted to GndRef
	MAX14920_reg_write(0b00000000,0b00000000,0b10000000|(cellTable[0]<<7));
	MAX14920_ReadData();
	
	// Testing Diag
	MAX14920_reg_write(0b00000000,0b00000000,0x00000100);
	Toggle_LED(7, 150, 4);
	_delay_ms(500);
	MAX14920_reg_write(0b00000000,0b00000000,0b00000110);
	_delay_ms(500); // Wait for voltage to be shifted to GndRef
	MAX14920_reg_write(0b00000000,0b00000000,0b00000000);
	MAX14920_ReadData();
	
	
	// Probing cell2
	//Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	//_delay_ms(500);
	//MAX14920_reg_write(0b00000000,0b00000000,0b11000100);	
	
	// Loop to hold processor
	while(1) {
		Toggle_LED(7, 1000,2);
		MAX14920_reg_write(0b00000000,0b00000000,0x00000100);
		//Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
		_delay_ms(500);
		MAX14920_reg_write(0b00000000,0b00000000,0b00000110);
		_delay_ms(500); // Wait for voltage to be shifted to GndRef
		MAX14920_reg_write(0b00000000,0b00000000,0b00000000);
		MAX14920_ReadData();
	}
}
