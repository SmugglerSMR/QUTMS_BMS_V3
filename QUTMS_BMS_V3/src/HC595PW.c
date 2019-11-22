/*
 * _74HC595PW.c
 *
 * Created: 22/11/2019 8:44:13 AM
 *  Author: sadykov
 */ 
#include "HC595PW.h"
#include "macros.h"
#include "CD74HCT4067.h"
#include "ADC.h"

uint8_t sensor_pattern[OVERALL_MESSAGE_PAIRS]={
	(I7U8 | I7U9),
	(I6U8 | I6U9),
	(I5U8 | I5U9),
	(I4U8 | I4U9),
	(I3U8 | I3U9),
	(I2U8 | I2U9),
	(I1U8 | I1U9),
	(I0U8 | I0U9),
	(I8U8 | I8U9),
	(I9U8 | I9U9),
	(I10U8 | I10U9),
	(I11U8 | I11U9),
	(I12U8 | I12U9),
	(I13U8 | I13U9),
	(I14U8 | I14U9),
	(I15U8 | I15U9)
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

void HC595PW_CD74HCT_send_read() {
	//Getting ADC value
	//uint16_t ADC_SensorA, ADC_SensorB, ADC_SensorC, ADC_SensorD;
	//SPI_send_byte((uint8_t)ADC_v);
	for(uint8_t i=0;i<OVERALL_MESSAGE_PAIRS;i++){
		//Write the data to HC595
		HC595PW_reg_write(sensor_pattern[i]);
		//_delays_ns(73+19);
		//ADC_SensorA = adc_read(9);
		//ADC_SensorB = adc_read(8);
		//ADC_SensorC = adc_read(2);
		//ADC_SensorD = adc_read(3);
		
		SensorTemp[i] = adc_read(9);
		SensorTemp[16+i] = adc_read(8);
		SensorTemp[32+i] = adc_read(2);
		SensorTemp[48+i] = adc_read(3);
		_delay_ms(10); //TODO: Get the smallest value
		
	}
}