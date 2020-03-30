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
#include "SPI.h"

#include "USART.h"
char floatStr[100];

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

void HC595PW_Init_Registers(void) {
	////Make the Data(DS), Shift clock (SH_CP), Store Clock (ST_CP) lines output
	DDRB |= ((1<<HC595PW_PIN_SH)|(1<<HC595PW_PIN_ST)|(1<<HC595PW_PIN_MR));	
	DDRC |= (1<<HC595PW_PIN_DS);
	
	//DDRD |= (~(1<<PIND5)|~(1<<PIND6));
	SET_INPUT(DDRD,PIND5); SET_INPUT(DDRD,PIND6);
	// Reset register Disable Reset
	WRITE_BIT(HC595PW_PORT_MR, HC595PW_PIN_MR, LOW);
	_delay_us(55);
	WRITE_BIT(HC595PW_PORT_MR, HC595PW_PIN_MR, HIGH);
	
	// TODO: Test if DS must be low
	WRITE_BIT(HC595PW_PORT_DS, HC595PW_PIN_DS, LOW);
		
}

//Sends a clock pulse on SH_CP || SCK line
void HC595Pulse(void) {
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
			 WRITE_BIT(HC595PW_PORT_DS, HC595PW_PIN_DS, HIGH);
		 } else {
			 //MSB is 0 so output high
			 WRITE_BIT(HC595PW_PORT_DS, HC595PW_PIN_DS, LOW);
		 }

		 HC595Pulse();  //Pulse the Clock line
		 data=data<<1;  //Now bring next bit at MSB position

	 }
	 //Now all 8 bits have been transferred to shift register
	 //Move them to output latch at one
	 HC595Latch();	
}

double HC595_CalcTemp(double resistance) {
	// 1/T = 1/T0 + 1/B * ln( R0 * ( ( adcMax / adcVal ) - 1 ) / R0 )
	// 1/T = 1/298.15 + 1/3380 * ln((1023 / 366) - 1 )
	// 1/T = 0.003527
	float steinhart;
	
	steinhart = resistance / THERMISTORNOMINAL;     // (R/Ro)
	steinhart = log(steinhart);                  // ln(R/Ro)
	steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
	steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
	steinhart = 1.0 / steinhart;                 // Invert
	steinhart -= 273.15;                         // convert to C
		
	return steinhart;
}
void HC595PW_CD74HCT_send_read(double CellResistance_One[], double CellResistance_Two[]) {
	
	// Reset Storage with latching
	WRITE_BIT(HC595PW_PORT_MR, HC595PW_PIN_MR, LOW);
	_delay_us(55);
	WRITE_BIT(HC595PW_PORT_MR, HC595PW_PIN_MR, HIGH);		
	HC595Latch();

	Max_Resistance = 10000;
	Min_Resistance = 10000;
	Average_Resistance = 10000;
	//Getting ADC value
	//uint16_t ADC_SensorA, ADC_SensorB, ADC_SensorC, ADC_SensorD;
	//SPI_send_byte((uint8_t)ADC_v);
	int ADC_IDs[2] = {2,3};
	double res_v[2] = {0};
	//uint16_t temp[4] = {0};
	Max_Resistance = 0;	
	for(int i=0;i<OVERALL_MESSAGE_PAIRS;i++){
		//HC595PW_reg_write(0x00);
		//_delay_us(55);
		//Write the data to HC595
		HC595PW_reg_write(sensor_pattern[i]);
		
		_delay_us(73+19);
		//_delay_ms(10);
		
		//Value of the resitance for each ADC
		// convert the value to resistance
		
		//reading = (1023 / reading)  - 1;     // (1023/ADC - 1)
		//reading = SERIESRESISTOR / reading;  // 10K / (1023/ADC - 1)		
		
		// TODO: VARIABLE REDECLARATION. Sampeling removed.
		// Combine loops
		//for(int j=0;j<SAMPLING;j++) 
		for(int k=0; k<2;k++)
			res_v[k] = RES_VALUE / ((1023.0 / (adc2_read(ADC_IDs[k]))) - 1);
			
			
			
			
		// Test if ADC samples						
		
		CellResistance_One[i] = HC595_CalcTemp(res_v[0]);
		CellResistance_Two[i] = HC595_CalcTemp(res_v[1]);
		//SPI_send_byte(steinhart3);
		//SensorTemp[16+i] = 0b00001111;
		//SensorTemp[32+i] = adc_read(2) >> 2;
		//SensorTemp[48+i] = adc_read(3) >> 2;
		//_delay_ms(10); //TODO: Get the smallest value
		
	}
}