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
#include <stdbool.h>
#include <stdio.h>

#include "macros.h"
#include "SPI.h"
#include "ADC.h"
#include "MAX14920.h"
#include "HC595PW.h"
#include "MCP2517FD.h"
#include "USART.h"

void IO_init(void);
void Toggle_LED(int id, int delay, int times);
uint16_t DecToBin(float nn);
char * FloatToStr(float x);

char floatStr[100];
float CellVoltages[11] = {0};
	
#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
(byte & 0x80 ? '1' : '0'), \
(byte & 0x40 ? '1' : '0'), \
(byte & 0x20 ? '1' : '0'), \
(byte & 0x10 ? '1' : '0'), \
(byte & 0x08 ? '1' : '0'), \
(byte & 0x04 ? '1' : '0'), \
(byte & 0x02 ? '1' : '0'), \
(byte & 0x01 ? '1' : '0')

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */
	// Initialize ATmega64M1 micro controller
	//board_init();
	IO_init();
	SPI_init();
	ADC_init();
	USART_1_init();
	
	MAX14920_Init_Registers();
	HC595PW_Init_Registers();
	//MCP2517FD_Init_Registers();
	
	// Alarm Line
	DDRC |= (1<<PINC1);
	WRITE_BIT(PORTC, PINC1, HIGH);
	
	// Initialize MAX14920 micro controller
	MAX14920_Clear_SPI_messages();
	MAX14920_Enable();
	// Perform Diagnostics
	//MAX14920_PerformDiagnosticsFirst();
	//MAX14920_PerformDiagnosticsSecond();
	
	// CAN MEssage
	//MCP2517_init();
	//BMS_BOARD_DATA[0] = 1;	//Board functionality	
	
	// Loop forever for checks	
	//uint8_t cycle = 0;
	//uint8_t data[8] = {0};
	//uint32_t receiveID = 0;
	//uint8_t numDataBytes;
	//PORTD ^= 0b00000001;
	
	unsigned char temp_avr[19] = "\n\rAverage Voltage: ";
	//unsigned char temp_max[19] = "\n\rMaximum Voltage: ";
	//unsigned char temp_min[19] = "\n\rMinimum Voltage: ";
	//unsigned char newline[2] = "\n\r";
	//
	////uart_init(BAUD_CALC(9600));
	//char st_avr[5] = "";
	//char st_max[5] = "";
	//char st_min[5] = "";
	//
	//char st_volt[4] = "";
	//char st_temp[4] = "";
	//LED4 - indicates success of MAX14920 init
	//LED5 - indicates success of HC595PW init
	//LED6 - indicates success of MCP2517FD init
	char *vSign;
	float vVal;
	int vInt1;
	float vFrac;
	int vInt2;
	
	float oVoltage = 0.0;
	
	while(1) {
		Toggle_LED(7, 500,1);
		///////////
		////snprintf(st_avr, 5, "%d", 17504);
		////snprintf(st_max, 5, "%d", 10400);
		////snprintf(st_min, 5, "%d", 98);
			////
		////////////////////////////////////////////
		// MAX14920  - Cell Manipulations
		// Do not read voltage during Cell balancing at all for now
		//if(~MAX14920_SPI_message.spiBalanceC01_C08 && ~MAX14920_SPI_message.spiBalanceC01_C08) {
		MAX14920_ReadAllCellsVoltage(CellVoltages);
		//for(int i=0; i<3; i++) {
			////snprintf(st_volt, 2, "%d", CellVoltages[i]);
			//// Try manually text
			//at64c1_transmit_str(ind_cell);
			////at64c1_transmit_str(st_volt);
			////_delay_ms(400);
		//}
		////_delay_ms(50);
		
		//OveralVoltage = MAX14920_ReadCellVoltage(0);
		//*vSign = (OveralVoltage < 0) ? "-" : "";
		//vVal = (OveralVoltage < 0) ? -OveralVoltage : OveralVoltage;
		//
		//vInt1 = vVal;                  // Get the integer (678).
		//vFrac = vVal - vInt1;      // Get fraction (0.0123).
		//vInt2 = trunc(vFrac * 10000);  // Turn into integer (123).
		//
		//sprintf (floatStr, "Overal Voltage:  %d.%04d \n", vInt1, vInt2);
		//at64c1_transmit_str(floatStr);
		//_delay_us(50);
		//printf();		
		//sprintf (floatStr, "Overall Voltage:  %d \n", OveralVoltage);
		//at64c1_transmit_str(floatStr);
		for(int i=0; i<10; i++) {
			oVoltage += CellVoltages[i];
			*vSign = (CellVoltages[i] < 0) ? "-" : "";
			vVal = (CellVoltages[i] < 0) ? -CellVoltages[i] : CellVoltages[i];
			
			vInt1 = vVal;                  // Get the integer (678).
			vFrac = vVal - vInt1;      // Get fraction (0.0123).
			vInt2 = trunc(vFrac * 10000);  // Turn into integer (123).			
			
			sprintf (floatStr, "CellVoltages %d:  %d.%04d \n", i, vInt1, vInt2);
			at64c1_transmit_str(floatStr);
			_delay_ms(50);
		}		
		
		*vSign = (oVoltage < 0) ? "-" : "";
		vVal = (oVoltage < 0) ? -oVoltage : oVoltage;
		
		vInt1 = vVal;                  // Get the integer (678).
		vFrac = vVal - vInt1;      // Get fraction (0.0123).
		vInt2 = trunc(vFrac * 10000);  // Turn into integer (123).
		
		sprintf (floatStr, "Overal Calculated Voltage:  %d.%04d \n\n", vInt1, vInt2);
		at64c1_transmit_str(floatStr);
		_delay_ms(50);
		oVoltage = 0.0;
		//sprintf (floatStr, "CellVoltages:  %d \n", CellVoltages[0]);
		//at64c1_transmit_str(floatStr);
		
		//// Adding cellcs
		//
		////}
		//
		//
		//// Report fault on any of the cells
		////if(~MAX14920_SPI_output.spiCellStatusC01_C08 ||
		   ////~MAX14920_SPI_output.spiCellStatusC09_C16) {
			////Toggle_LED(5,500,1);
		////}
		//
		//// Toggle balancer
		//// TODO: Recheck values before playing with balancer
		//// TODO: Make sure that values for threshold is accurate first
		//// This one only for charge.
		////MAX14920_EnableLoadBalancer(true);
		//
		////////////////////////////////////////////
		//// 74HC595PW - Start Temperature readings
		//// TODO: Record temperature for CAN
		HC595PW_CD74HCT_send_read();
		////for(int i=0;i<32;i++) {
			////at64c1_transmit_str("\n\rSensor Resistance: ");
			////snprintf(st_volt, 5, "%d", CellVoltages[i]);
			//////_delay_ms(400);
		////}
		//
		////////////////////////////////////////////
		//// MCP2517FD - CANBUS		
		////BMS_BOARD_DATA[0] = OveralVoltage;	//Board functionality
		////BMS_BOARD_DATA[1] = AverageCellVoltage;	//Board functionality
		////BMS_BOARD_DATA[2] = Max_Resistance;	//Board functionality
		////BMS_BOARD_DATA[4] = CellVoltages[0];	//Board functionality
		////BMS_BOARD_DATA[5] = cycle;	//Board functionality
		//
		//SPI_send_byte(0b11111111);
		//SPI_send_byte((uint8_t)(OveralVoltage>>8));
		//SPI_send_byte((uint8_t)OveralVoltage);
		//SPI_send_byte(0b11111111);
		//SPI_send_byte((uint8_t)(AverageCellVoltage>>8));
		//SPI_send_byte((uint8_t)AverageCellVoltage);
		//SPI_send_byte(0b11111111);
		//SPI_send_byte((uint8_t)(Max_Resistance>>8));
		//SPI_send_byte((uint8_t)Max_Resistance);
		//SPI_send_byte(0b11111111);
		//SPI_send_byte((uint8_t)(Min_Resistance>>8));
		//SPI_send_byte((uint8_t)Min_Resistance);
		//SPI_send_byte(0b11111111);
		//_delay_ms(100);
		//SPI_send_byte(0b11111111);
		//SPI_send_byte(0b11111111);
		//SPI_send_byte(0b11111111);
		
		// Checking status
		MAX14920_EnableHoldPhase(true);
		MAX14920_reg_write();
		_delay_us(10);
		
		sprintf (floatStr, "\nCellStatusC01_C08: "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(MAX14920_SPI_output.spiCellStatusC01_C08));
		at64c1_transmit_str(floatStr);
		_delay_us(50);
		sprintf (floatStr, "\nCellStatusC09_C16: "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(MAX14920_SPI_output.spiCellStatusC09_C16));
		at64c1_transmit_str(floatStr);
		_delay_us(50);
		sprintf (floatStr, "\nChipStatus: "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(MAX14920_SPI_output.spiChipStatus));
		at64c1_transmit_str(floatStr);
		_delay_us(50);
		sprintf (floatStr, "\n\n");
		at64c1_transmit_str(floatStr);
		MAX14920_EnableHoldPhase(false);
		MAX14920_reg_write();
		
		for(int i=0;i<10;i++) {
			SPI_send_byte((uint8_t)CellVoltages[i] >>8);
			SPI_send_byte((uint8_t)CellVoltages[i]);
			if(CellVoltages[i] < 3100 ) WRITE_BIT(PORTC,PINC1,LOW);
			if(CellVoltages[i] > 3500 ) WRITE_BIT(PORTC,PINC1,LOW);
		}		
		//SPI_send_byte(0b11111111);
		//SPI_send_byte(0b11111111);
		//SPI_send_byte(0b11111111);
		for(int i=0;i<32;i++) {
			SPI_send_byte((uint8_t)CellResistance_One[i] >>8);
			SPI_send_byte((uint8_t)CellResistance_One[i]);
			_delay_us(5);
			SPI_send_byte((uint8_t)CellResistance_Two[i] >>8);
			SPI_send_byte((uint8_t)CellResistance_Two[i]);
			if(CellResistance_One[i] < 3849 ||
			   CellResistance_Two[i] < 3849 ) WRITE_BIT(PORTC,PINC1,LOW);
		}
		
		//if(BIT_IS_SET(PORTC, PINC1)) WRITE_BIT(PORTC,PINC1,HIGH);
		
		//////_delay_ms(500);
		//////SPI_send_byte((uint8_t)receiveID >> 24);
		//////SPI_send_byte((uint8_t)receiveID >> 16);
		//////SPI_send_byte((uint8_t)receiveID >> 8);
		//////SPI_send_byte((uint8_t)receiveID);
		////
		//
		//MCP2517_recieveMessage(&receiveID, &numDataBytes, data);
		//if(receiveID == CAN_ID_PDM >> 18) {
			//PORTC ^= 0b00000001;
			//MCP2517_transmitMessage(CAN_ID_AMS, 2, BMS_BOARD_DATA);
			////_delay_ms(5000);
			////Toggle_LED(5,1000,1);
			//
			//// TODO: Think about message comparison
		//}
		////
		//MCP2517_transmitMessage(CAN_ID_AMS, 2, BMS_BOARD_DATA);	
		////cycle++;
		////if(cycle >=200)
			//cycle = 0;
		//
		//
		//at64c1_transmit_str(temp_avr);
	}
}

void IO_init(void) {
	// Initialize LEDs
	DDRB = 0b00011000;	// LED-5 LED-4
	DDRC = 0b00000001;	// LED-3 
	DDRD = 0b00000011;	// LED 7 6
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
				for (int i = 0; i < delay/2; i++)	{
					_delay_ms(1);
				}
				PORTB ^= 0b00010000;
				break;			
			case 4:		// blue
				PORTB ^= 0b00001000;
				for (int i = 0; i < delay/2; i++)	{
					_delay_ms(1);
				}
				PORTB ^= 0b00001000;
				break;
			case 3:		// blue
				PORTC ^= 0b00000001;
				for (int i = 0; i < delay/2; i++)	{
					_delay_ms(1);
				}
				PORTC ^= 0b00000001;
				break;		
			case 7:		// white
				PORTD ^= 0b00000010;
				for (int i = 0; i < delay/2; i++)	{
					_delay_ms(1);
				}
				PORTD ^= 0b00000010;
				break;
			case 6:		// red
				PORTD ^= 0b00000001;
				for (int i = 0; i < delay/2; i++)	{
					_delay_ms(1);
				}
				PORTD ^= 0b00000001;
				break;
		}
		for (int i = 0; i < delay/2; i++)	{
			_delay_ms(1);
		}
	}
}

uint16_t DecToBin(float nn) {
	int a[8];
	uint8_t byte = 0;
	int n = (int) nn;	
	for(int i=0;n>0;i++){
		a[i]=n%2;
		n=n/2;
	}
	for(int i=0;n>8;i++){
		if(a[i] == 1) {
			byte |= 1;
			byte <<=1;
		} else {
			byte |= 0;
			byte <<=1;
		}
	}
	return byte;
}

char * FloatToStr(float x) {

	char *tmpSign = (x < 0) ? "-" : "";
	float tmpVal = (x < 0) ? -x : x;

	int tmpInt1 = tmpVal;                  // Get the integer (678).
	float tmpFrac = tmpVal - tmpInt1;      // Get fraction (0.0123).
	int tmpInt2 = trunc(tmpFrac * 10000);  // Turn into integer (123).

	sprintf (floatStr, "%s%d.%04d\n", tmpSign, tmpInt1, tmpInt2);
	
	return floatStr;
}