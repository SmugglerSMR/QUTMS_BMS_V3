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
	/* Insert system clock initialization code here (sysclk_init()). */
	DDRB = 0b10110000; // MOSI and SCK and SS output
	
	DDRA = 0b01010011;	//CS_0 CS_1 Pins high
	
	WRITE_BIT(CAN_CS_PORT,CAN_CS_0,HIGH);
	WRITE_BIT(CAN_CS_PORT,CAN_CS_1,HIGH);	
	
	uart0_init(9600);
	spi_init(1,0); //0,0
	//_delay_ms(50);
	//MCP2517_init(CAN_CS_0);
	//MCP2517_init(CAN_CS_1);
	//PORTB |= (1<<PINB4);
	
	//PORTB &= ~(1<<PINB5);
	//_delay_ms(1000);
	/* Insert application code here, after the board has been initialized. */
	AMS_BOARD_DATA[0] = 25;
	AMS_BOARD_DATA[1] = 530;
	//AMS_BOARD_DATA[2] = 0x02;
	//AMS_BOARD_DATA[3] = 0x03;
	//AMS_BOARD_DATA[4] = 0x04;
	
	//uint8_t data[8] = {0};
	//uint32_t receiveID;
	//uint8_t numDataBytes;
	unsigned char temp_len = 25;
	unsigned char temp_str[25] = "\n\rAverage Temperature: ";
	
	unsigned char volt_len = 20;
	unsigned char volt_str[20] = "\n\rOveral Voltage: ";
	
	char st_Ebf[4] = "";
	char st_int[5] = "";
	float Eb_float = 0;
	Eb_float = (float)AMS_BOARD_DATA[1] / 100;
	snprintf(st_int, 5, "%d", AMS_BOARD_DATA[0] );
	//sprintf(st_Ebf, "%f", Eb_float);
	formatTempString(st_Ebf, Eb_float);
	while(1) {
		PORTA ^= 0b01000000;
		
		//MCP2517_transmitMessage(CAN_ID_BMS, 5, AMS_BOARD_DATA, CAN_CS_0);
		//MCP2517_transmitMessage(CAN_ID_BMS, 5, AMS_BOARD_DATA, CAN_CS_1);
		//_delay_ms(500);
		//	spi_send_byte(0b00011000);
		//CAN_CS=CAN_CS_A;
		
		//AMS_BOARD_DATA[0] = 0x00+1;
		//if(AMS_BOARD_DATA[0] > 200)
		//AMS_BOARD_DATA[0] = 0x00;
		//_delay_ms(50);
		//MCP2517_recieveMessage(&receiveID, &numDataBytes, data);
		//if(receiveID == CAN_ID_AMU >> 18) {
		//spi_send_byte(0b00111100);
		//_delay_ms(50);
		//spi_send_byte(0b00111100);
		//}
		////
		//
		//if(receiveID == CAN_ID_AMU >> 18) {
		//MCP2517_transmitMessage(CAN_ID_AMU, 5, AMS_BOARD_DATA);
		//spi_send_byte(0b11111111);spi_send_byte(0b11111111);spi_send_byte(0b11111111);
		//_delay_ms(50);
		//}
		//MCP2517_recieveMessage(&receiveID, &numDataBytes, data, CAN_CS_0);
		//if(receiveID == CAN_ID_PDM >> 18) {
		////if(receiveID == CAN_ID_PDM) {
			////PORTC ^= 0b00001000;
			//spi_send_byte((uint8_t)receiveID >> 24);
			//spi_send_byte((uint8_t)receiveID >> 16);
			//spi_send_byte((uint8_t)receiveID >> 8);
			//spi_send_byte((uint8_t)receiveID);
			////spi_send_byte(0b11111111);spi_send_byte(0b11111111);spi_send_byte(0b11111111);
			//_delay_ms(50);
		//}
		//_delay_ms(20);
		//
		//MCP2517_recieveMessage(&receiveID, &numDataBytes, data, CAN_CS_1);
		//if(receiveID == CAN_ID_PDM >> 18) {
			////PORTC ^= 0b00001000;
			//spi_send_byte((uint8_t)receiveID >> 24);
			//spi_send_byte((uint8_t)receiveID >> 16);
			//spi_send_byte((uint8_t)receiveID >> 8);
			//spi_send_byte((uint8_t)receiveID);
			//
			//_delay_ms(50);
		//}
		
		_delay_ms(500);
		uart_transmit_str(temp_str);
		uart_transmit_str(st_int);
		uart_transmit_str(volt_str);
		uart_transmit_str(st_Ebf);
		
		
		//PORTB ^= 0b00100000;
	}
}