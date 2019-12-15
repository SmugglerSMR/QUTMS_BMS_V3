#define F_CPU 16000000UL

#include <asf.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "input.h"
#include "UART.h"
#include "SPI.h"
#include "MCP2517.h"
#include "macros.h"

void formatTempString (char *s, float temp);
float HC595_CalcTemp(uint16_t resistance);

uint8_t AMS_BOARD_DATA[5] = {0};

/*============================================================================
Function:   AMS_init()
------------------------------------------------------------------------------
Purpose :   consolidates all the functions required for the initialisation of
			the AMU board
Input   :   none
Returns :   void
Notes   :
============================================================================*/
void AMS_init() {
	//_delay_ms(500);
	
	DDRB = 0b10110000; // MOSI and SCK as output, SS output
	//DDRA = 0b01010001; // LEDS and CAN_CS 
	DDRA = 0b00000001; // LEDS and CAN_CS 
	// Set Pre-charge, Shutdown+ and Shutdown- as outputs
	DDRD = 0b00001111;
	DDRC = 0b00000000;
	
	PORTD |=(1<<PIND0) | ~(1<<PIND1) | ~(1<<PIND2) | ~(1<<PIND3);
	//PORTD |=(1<<PIND0) | (1<<PIND1) | (1<<PIND2) | (1<<PIND3);
	//PORTD = 0b00000001;
	CAN_CS_PORT |= (1 << CAN_CS); // CS high to turn off
	
	//SHDN_NEG_OFF;
	//SHDN_POS_OFF;
	//PRE_CHARGE_OFF;
	
	//adc_init();
	//uart0_init(9600);
	spi_init(0,0); // 1,0
	MCP2517_init();
	
	sei(); // Enable interrupts
}

void formatTempString (char *s, float temp) {
	int t0,t0d;

	t0= (int) (temp/100);
	t0d= (int) ((temp)-(t0*100));

	snprintf (s, 4, "%d.%d", t0, t0d);
}

//NCP18XH103F03RB - Related variables
//#define RES_VALUE 10000
#define THERMISTORNOMINAL 10000
//+1.7K is the offset
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3380
float HC595_CalcTemp(uint16_t resistance) {
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
	
	//*(float*)(bytes) = steinhart*10;  // convert float to bytes
	//SPI_send_byte((uint8_t)bytes[0]);
	//SPI_send_byte((uint8_t)bytes[1]);
	//SPI_send_byte((uint8_t)bytes[2]);
	//SPI_send_byte((uint8_t)bytes[3]);
	
	if(steinhart >= 50.0 || steinhart <= 5)
		PORTC ^= 0b00000001; // Indicate cold om LED3
	return steinhart;
}

int main (void)
{
	AMS_init();
	   
	uint8_t data[8] = {0};
	CAN_RECEIVE_ADDRESS receiveID;
	uint8_t numDataBytes;
	//WRITE_BIT(PORTD, PIND0, HIGH);
	/* Insert application code here, after the board has been initialized. */
	//AMS_BOARD_DATA[0] = 25;
	//AMS_BOARD_DATA[1] = 530;
	//AMS_BOARD_DATA[2] = 0x02;
	//AMS_BOARD_DATA[3] = 0x03;
	//AMS_BOARD_DATA[4] = 0x04;
	
	//uint8_t data[8] = {0};
	//uint32_t receiveID;
	//uint8_t numDataBytes;
	//unsigned char temp_len = 25;
	//unsigned char temp_str[25] = "\n\rAverage Temperature: ";
	//
	//unsigned char volt_len = 20;
	//unsigned char volt_str[20] = "\n\rOveral Voltage: ";
	//
	//char st_Ebf[4] = "";
	//char st_int[5] = "";
	//float Eb_float = 0;
	//Eb_float = (float)AMS_BOARD_DATA[1] / 100;
	//snprintf(st_int, 5, "%d", AMS_BOARD_DATA[0] );
	////sprintf(st_Ebf, "%f", Eb_float);
	//formatTempString(st_Ebf, Eb_float);
	_delay_ms(500);
	//PRE_CHARGE_ON;
	//SHDN_POS_ON;
	//SHDN_NEG_ON;
	while(1)
	{
		// Fan control pin is BMS alarm
		MCP2517_recieveMessage(&receiveID, &numDataBytes, data);
		if(receiveID == CAN_RECEIVE_ID_PDM >> 18) { // Use PDM CAN packet
			/* Byte 0 */
			//LED_A_ON;
			if(CHECK_BIT(data[0], 6)) { // Shutdown -
			//if(first) DDRB |=()|();
				PRE_CHARGE_ON;
				SHDN_NEG_ON;
			} else {
				PRE_CHARGE_OFF;
			//SHDN_NEG_OFF;
			}
			if(CHECK_BIT(data[0], 7)) { // Shutdown +
				SHDN_POS_ON;
				//LED_A_ON;
			} else {
				SHDN_POS_OFF;
				//LED_A_OFF;
			}
		} else {
			PRE_CHARGE_OFF;
			SHDN_POS_OFF;
			//LED_A_OFF;
		} 
		
		//LED_A_TOGGLE;
		_delay_ms(100);
		receiveID = 0;
		if(BIT_IS_SET(PORTC, PINC4)) WRITE_BIT(PORTD, PIND0, LOW);
	}
}