/**
 * \file
 *
 * \brief Empty user application template
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
#include "MCP2517.h"
#include "macros.h"
uint8_t AMS_BOARD_DATA[5] = {0};
	
int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	DDRB = 0b10110000;	//SS-output, MOSI-out, CLK-out
	
	DDRA = 0b00000011;	//CD Pins high
	
	WRITE_BIT(PORTA,PINA1,HIGH);
	WRITE_BIT(PORTA,PINA0,HIGH);
	//PORTA |= (1<<PINA1); // Disable CS	
	//PORTA |= (1<<PINA0); // Disable CS	
	//board_init();
	
	spi_init(1,0);
	_delay_ms(50);
	MCP2517_init();
	//PORTB |= (1<<PINB4);
	
	//PORTB &= ~(1<<PINB5);
	_delay_ms(1000);
	/* Insert application code here, after the board has been initialized. */
	AMS_BOARD_DATA[0] = 0x00;
	AMS_BOARD_DATA[1] = 0x01;
	AMS_BOARD_DATA[2] = 0x02;
	AMS_BOARD_DATA[3] = 0x03;
	AMS_BOARD_DATA[4] = 0x04;
	
	uint8_t data[8] = {0};
	uint32_t receiveID;
	uint8_t numDataBytes;
	
	MCP2517_transmitMessage(CAN_ID_AMU, 5, AMS_BOARD_DATA);
	_delay_ms(500);
	while(1) {
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
		MCP2517_recieveMessage(&receiveID, &numDataBytes, data);
		if(receiveID == CAN_ID_PDM >> 18) {
			PORTC ^= 0b00001000;
			//SPI_send_byte(0b11111111);SPI_send_byte(0b11111111);SPI_send_byte(0b11111111);
			_delay_ms(50);
		}
		_delay_ms(20);
		//PORTB ^= 0b00100000;
	}
}
