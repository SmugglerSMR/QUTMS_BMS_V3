/*
 * _74HC595PW.c
 *
 * Created: 22/11/2019 8:44:13 AM
 *  Author: sadykov
 */ 
#include "HC595PW.h"
#include "macros.h"

uint8_t sensor_pattern[8]={
	0b10000001,
	0b11000011,
	0b11100111,
	0b11111111,
	0b01111110,
	0b00111100,
	0b00011000,
	0b00000000,
};

void HC595PW_init() {
	////Make the Data(DS), Shift clock (SH_CP), Store Clock (ST_CP) lines output
	//HC595_DDR|=((1<<HC595_SH_CP_POS)|(1<<HC595_ST_CP_POS)|(1<<HC595_DS_POS));
}

//Sends a clock pulse on SH_CP || SCK line
void HC595Pulse() {
	//Pulse the Shift Clock
	//HC595_PORT|=(1<<HC595_SH_CP_POS);//HIGH
	//WRITE_BIT(HC595PW_PORT_SH, HC595PW_PIN_SH, HIGH);
	SET_BIT(HC595PW_PORT_SH, HC595PW_PIN_SH);
	
	//HC595_PORT&=(~(1<<HC595_SH_CP_POS));//LOW
	//WRITE_BIT(HC595PW_PORT_SH, HC595PW_PIN_SH, LOW);
	CLEAR_BIT(HC595PW_PORT_SH, HC595PW_PIN_SH);
}

//Sends a clock pulse on ST_CP || ST_CLK line
void HC595Latch() {
	//Pulse the Store Clock
	//HC595_PORT|=(1<<HC595_ST_CP_POS);//HIGH
	//WRITE_BIT(HC595PW_PORT_ST, HC595PW_PIN_ST, HIGH);
	SET_BIT(HC595PW_PORT_ST, HC595PW_PIN_ST);
	_delay_loop_1(1);

	//HC595_PORT&=(~(1<<HC595_ST_CP_POS));//LOW
	//WRITE_BIT(HC595PW_PORT_ST, HC595PW_PIN_ST, LOW);
	CLEAR_BIT(HC595PW_PORT_ST, HC595PW_PIN_ST);
	_delay_loop_1(1);
}

/*
	Main High level function to write a single byte to
	Output shift register 74HC595.

	Arguments:
	single byte to write to the 74HC595 IC

	Returns:
	NONE

	Description:
	The byte is serially transfered to 74HC595
	and then latched. The byte is then available on
	output line Q0 to Q7 of the HC595 IC.
	
	Example based on: http://extremeelectronics.co.in/avr-tutorials/using-shift-registers-with-avr-micro-avr-tutorial/
*/
void HC595PW_reg_write(uint8_t data){
	 //Send each 8 bits serially

	 //Order is MSB first
	 for(uint8_t i=0;i<8;i++) {
		 //Output the data on DS line according to the
		 //Value of MSB
		 if(data & 0b10000000) {
			 //MSB is 1 so output high
			 WRITE_BIT(HC595PW_PORT_DC, HC595PW_PIN_DC, HIGH);
		 } else {
			 //MSB is 0 so output high
			 WRITE_BIT(HC595PW_PORT_DC, HC595PW_PIN_DC, LOW);
		 }

		 HC595Pulse();  //Pulse the Clock line
		 data=data<<1;  //Now bring next bit at MSB position

	 }
	 //Now all 8 bits have been transferred to shift register
	 //Move them to output latch at one
	 HC595Latch();	
}

void HC595PW_CD74HCT_send() {
	for(uint8_t i=0;i<8;i++){
		//Write the data to HC595
		HC595PW_reg_write(sensor_pattern[i]);
		_delay_ms(500);
	}
}