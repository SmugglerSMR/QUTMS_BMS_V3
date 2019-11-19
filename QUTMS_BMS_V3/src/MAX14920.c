/*
 * MAX14920.c
 *
 * Created: 19/11/2019 3:48:20 PM
 *  Author: sadykov
 */ 
#include "MAX14920.h"
#include "macros.h"
#include "SPI.h"
#include "ADC.h"

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

void MAX14920_Enable(void) {

	//PORTC |= (1<<PINC6); // Set SAMPL high to track voltage at CV
//	PORTD |= (1<<PIND7); // SET EN High to enable device shutdown mode. It resets the SPI register, make sure it's on
	_delay_ms(1);
	//MAX14920_PORT_CS &= ~(1<<MAX14920_PIN_CS); // unset to start transmission
	//_delay_ms(1);
	//SPI_send_byte(0x00);
	//SPI_send_byte(0x00);
	//SPI_send_byte(0b00000000);
	//
	//_delay_ms(1);
	//MAX14920_PORT_CS |= 1<<MAX14920_PIN_CS; // Set back.
	//_delay_ms(10);

	SPI_send_byte(0x00); // fIRST 8 BIT NEVER GETS SEND
	uint8_t status = 0b00000010;
	while((status&0b00000010)) {
		//PORTD |= (1<<PIND7); // SET EN High to enable device shutdown mode. It resets the SPI register, make sure it's on
		_delay_ms(1);

		MAX14920_PORT_CS &= ~(1<<MAX14920_PIN_CS); // unset to start transmission
		SPI_send_byte(0x00);
		SPI_send_byte(0b00000000);
		status = SPI_send_byte(0b00000100);
		MAX14920_PORT_CS |= 1<<MAX14920_PIN_CS; // Set back.
		_delay_ms(10);
		Toggle_LED(5, 500, 1);
		PORTD &= ~(1<<PIND7); // SET EN High to enable device shutdown mode. It resets the SPI register, make sure it's on
		_delay_ms(1);

	}
}

void MAX14920_ReadData(void) {
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
	SPI_send_byte((uint8_t)ADC_v);

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
