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
void IO_init() {
	// Initialise LEDs
	DDRB = 0b10011010;	// CLC-outout LED 5 4 MOSI-output MISO-input
	DDRC = 0b00001001;	// CS-high LED 3 
	DDRD = 0b00001011;	// SS-high LED 7 6
	
	//PORTC |= (1<<PINC3); // Set SS as output high
	//PORTB |= (1<<PINB1); //SET MOSi as output
	// TODO: COMLETE THOSE PARTS
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
			for (int i = 0; i < delay; i++)	{
				_delay_ms(1);
			}
			PORTB ^= 0b00000000;
			break;			
		case 4:		// blue
			PORTB ^= 0b00001000;
			for (int i = 0; i < delay; i++)	{
				_delay_ms(1);
			}
			PORTB ^= 0b00000000;
			break;
		case 3:		// blue
			PORTC ^= 0b00000000;
			for (int i = 0; i < delay; i++)	{
				_delay_ms(1);
			}
			PORTC ^= 0b00000000;
			break;		
		case 7:		// white
			PORTD ^= 0b00000010;
			for (int i = 0; i < delay; i++)	{
				_delay_ms(1);
			}
			PORTD ^= 0b00000000;
			break;
		case 6:		// red
			PORTD ^= 0b00000001;
			for (int i = 0; i < delay; i++)	{
				_delay_ms(1);
			}
			PORTD ^= 0b00000000;
			break;
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

void ADC_init() {
	// AVcc with capacitor
	// !!!! If any inaccuaracies accured, choise option without capacitor
	ADMUX = (1<<REFS0)| (1<<AREFEN);	
	ADMUX &= ~(1 << ADLAR); // Ôîãûåüóòå
	// 16MHz clock/128 prescaler= 125kHz = 0.000008s.
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC6_read() {
	// Set to PB5 or ADC6 | REFS0 defines AVcc with no external capacitor connected on the AREF pin
	ADMUX = (1<<REFS0)| (1<<ADLAR) | (1<<MUX2) | (1<<MUX1);
	
	// Start single conversion by setting ADSC bit in ADCSRA
	ADCSRA |= (1 << ADSC);
	while ( ADCSRA & (1 << ADSC) ) {}
	return ADC;
}

/*============================================================================
Function:   adc_read()
------------------------------------------------------------------------------
Purpose :   reads an analog input voltage and converts it to a 10-bit digital
			value through successive approximation
Input   :   uint8_t channel - selected analog input channel
Returns :   result - pass the 10 bit ADC number to requesting function
Notes   :
============================================================================*/
uint16_t adc_read(uint8_t channel) 
{   
    channel = (ADMUX & 0xe0)|(channel & 0x1F); /* only change ADMUX bits signalling which channel to use */
	ADMUX = channel;
	//SET_BIT(ADCSRA, ADSC); /* start conversion process */
	ADCSRA |= (1<<ADSC);
	//while(!(CHECK_BIT(ADCSRA, ADIF))); /* loop while the conversion is taking place */
	while ( ADCSRA & (1 << ADSC) ) {}
	uint16_t result = 0;	
	result = ADCL; /* read ADCL, then ADCH --> order is important! */							
	result |= ((3 & ADCH) << 8);
	//--> also not sure if this code is correct. other ADC examples return 'ADC' instead. //
	//SET_BIT(ADCSRA, ADIF); /* clear 'complete' status */
	ADCSRA |= (1<<ADIF);
	return result;
}

int * DecToBin(double nn) {
	static int a[5];
	int n = (int) nn;	
	for(int i=0;n>0;i++){
		a[i]=n%2;
		n=n/2;
	}
	return a;
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
	// ADC output is next thing to do
	// Try get response or similar
	int bit_test = 0;
	//ADC0_BASE
	// TEST LINE for blicking
	//output = 0b0001000;
	
	
	//Getting ADC value
	//uint16_t ADC_v = ADC6_read();
	
	//uint16_t ADC_v = adc_read(6);
	//double res_voltage = ADC_v;
	//double res_voltage = 600/(double)ADC_v;
	//int *p = DecToBin(res_voltage);
	//for(int i = 0; i < 5;i++) if(p[i]==1) Toggle_LED(3+i,3000);
	//if(a[0] == 1) Toggle_LED(3,3000);
	//if(a[1] == 1) Toggle_LED(4,3000);
	//if(a[2] == 1) Toggle_LED(5,3000);
	//if(a11)
	
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
	board_init();
	IO_init();
	SPI_init();
	ADC_init();
	/* Insert application code here, after the board has been initialized. */
	_delay_ms(3000);
	
	// Probing general
	Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	_delay_ms(500);
	MAX14920_reg_write(0x00,0x00,0x02);
	
	
	// Probing CEL1
	Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	_delay_ms(500);
	MAX14920_reg_write(0x00,0x00,0x84);
	
	// Probing CEL1
	Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	_delay_ms(500);
	MAX14920_reg_write(0x00,0x00,0x84);
	
	
	// Probing cell2
	Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	_delay_ms(500);
	MAX14920_reg_write(0x00,0x00,0xC4);
	
	// Probing cell2
	Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	_delay_ms(500);
	MAX14920_reg_write(0x00,0x00,0xC4);
		
	// Probing cell2
	Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	_delay_ms(500);
	MAX14920_reg_write(0x00,0x00,0xC4);
	
	// Probing CEL1
	Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	_delay_ms(500);
	MAX14920_reg_write(0x00,0x00,0x84);
	
	// Probing CEL1
	Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	_delay_ms(500);
	MAX14920_reg_write(0x40,0x00,0x84);
	
	
	//
	//// Probing cell3
	//Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	//_delay_ms(500);
	//MAX14920_reg_write(0x00,0x00,0xA4);
	//
	//// Probing cell4
	//Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	//_delay_ms(500);
	//MAX14920_reg_write(0x00,0x00,0xE4);
	//
	//// Probing cell5
	//Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	//_delay_ms(500);
	//MAX14920_reg_write(0x00,0x00,0xE4);
	//
	// Probing general
	//Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	//_delay_ms(500);
	//MAX14920_reg_write(0x00,0x00,0x02);
	//// Probing general
	//Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	//_delay_ms(500);
	//MAX14920_reg_write(0x00,0x00,0x02);
	//// Probing general
	//Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
	//_delay_ms(500);
	//MAX14920_reg_write(0x00,0x00,0x02);
	// Loop to hold processor
	while(1) {
		Toggle_LED(7, 2000);
		Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
		_delay_ms(500);
		//MAX14920_reg_write(0x01,0x00,0x02);
		MAX14920_reg_write(0x00,0x30,0x02);
		//Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);Toggle_LED(7, 150);
		//_delay_ms(500);
		//MAX14920_reg_write(0x00,0x00,0xC4);
		
	}
}
