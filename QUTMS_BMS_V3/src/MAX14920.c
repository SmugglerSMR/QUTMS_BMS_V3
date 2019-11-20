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

static const int cellTable[MAX14920_CELL_NUMBER] = {
	0b0000, 0b1000, 0b0100, 0b1100,	// Cells 1, 2, 3, 4
	0b0010, 0b1010, 0b0110, 0b1110,	// Cells 5, 6, 7, 8
	0b0001, 0b1001, 0b0101, 0b1101	// Cells 9,10,11,12
};

void MAX14920_reg_write(uint8_t CB1_CB8, uint8_t CB9_CB16, uint8_t ECS) {
	uint8_t output;
	
	//PORTC &= ~(1<<PINC6); // Sampl_disable
	//_delay_ms(50);
	//
	//PORTC &= ~(1<<PINC6); // Sampl_disable
	
	// unset to start transmission
	WRITE_BIT(MAX14920_PORT_CS, MAX14920_PIN_CS, LOW);
	//TODO: Make sure that extra time is needed
	_delay_us(100);
	
	SPI_send_byte(CB1_CB8);
	SPI_send_byte(CB9_CB16);
	output = SPI_send_byte(ECS);
	
	// Set back.
	WRITE_BIT(MAX14920_PORT_CS, MAX14920_PIN_CS, HIGH);
	//TODO: Make sure that extra time is needed
	_delay_us(80);
}

void MAX14920_Enable(void) {

	//PORTC |= (1<<PINC6); // Set SAMPL high to track voltage at CV	
	// SET EN High to enable device shutdown mode. 
	//	It resets the SPI register, make sure it's on
	SET_BIT(MAX14920_PORT_EN, MAX14920_PIN_EN);
	_delay_ms(1+8);	//+8for calibration

	SPI_send_byte(0x00); // fIRST 8 BIT NEVER GETS SEND
	uint8_t status = 0b00000010;
	while((status&0b00000010)) {		
		MAX14920_reg_write(0x00,0x00,0x00);
				
		// Wait a bit before try again.
		Toggle_LED(5, 100, 1);
		//PORTD &= ~(1<<PIND7); // SET EN High to enable device shutdown mode. It resets the SPI register, make sure it's on
		//_delay_ms(1);

	}
}

void MAX14920_OffsetCallibration(){
	// Set SAMPL high to start calibration
	WRITE_BIT(MAX14920_PORT_SMPLB, MAX14920_PIN_SMPLB, HIGH);
	
	MAX14920_reg_write(0x00,0x00,0b01000000);
	_delay_ms(8);
	// Set SAMPL back to low to as not necessary.
	WRITE_BIT(MAX14920_PORT_SMPLB, MAX14920_PIN_SMPLB, LOW);
}

void MAX14920_ReadData(void) {
	PORTC &= ~(1<<PINC6); // Set SAMPL low to make ready
	// TODO: ADC output is next thing to do
	_delay_us(100);
	
	// Try get response or similar
	int bit_test = 0;
	
	//ADC0_BASE
	// TEST LINE for blicking
	//output = 0b0001000;
	
	// Send dummy value
	MAX14920_reg_write(0x00,0x00,0x00);
	
	
	//Getting ADC value
	uint16_t ADC_v = adc_read(6);
	
	PORTC |= (1<<PINC6); // Set SAMPL high to track voltage at CV
	SPI_send_byte((uint8_t)ADC_v);
	
	// AOUT settling time is 60us
	_delay_us(60);
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
//TODO! Nees a structure, to keep what balancing command has been sent already, as a global variable.
void MAX14920_ReadCellVoltage(int cellN) {
	
	// Disable Sampler??
	//SET_BIT(MAX14920_PORT_CS, PINC6);
	
	if(cellN !=0) {
		WRITE_BIT(MAX14920_PORT_CS, PINC6, LOW);
		MAX14920_reg_write(0x00, 0x00,0x80||(cellTable[cellN-1]<<7));
		_delay_ms(10);
		MAX14920_ReadData();
	}
}

void MAX14920_ReadAllCellsVoltage() {
	for (int cellI = 0; cellI<MAX14920_CELL_NUMBER; cellI++) {
		
	}
}