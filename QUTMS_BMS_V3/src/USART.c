/*
 * USART.c
 *
 * Created: 05.12.2019 8:39:52
 *  Author: User
 */
#include "USART.h" 

// ********************************* Initialisation USART *********************************

void USART_1_init(void) {
	LINCR = (1 << LSWRES);
	//LINBRRH = (((F_CPU/BAUD)/16)-1)>>8;
	//LINBRRL = (((F_CPU/BAUD)/16)-1);
	LINBRRH = 0;
	LINBRRL = 51;
	
	LINBTR = (1 << LDISR) | (16 << LBT0);
	LINCR = (1<<LENA)|(1<<LCMD2)|(1<<LCMD1)|(1<<LCMD0);
}

//* ************************************* USART ATMEGA64C1 Tx**********************************
 int at64c1_transmit (unsigned char byte_data) {
    while (LINSIR & (1 << LBUSY));          // Wait while the UART is busy.
    LINDAT = byte_data;
    return 0;
}

void at64c1_transmit_str(unsigned char* str) {
	while (*str) // keep going until NULL terminator found
		at64c1_transmit(*str++);
}

//void at64c1_transmit_byte(uint8_t value) {
	////char *vSign;
	//float vVal;
	//int vInt1;
	//float vFrac;
	//int vInt2;	
	//char floatStr[100];
	//
	////*vSign = (value < 0) ? "-" : "";
	//vVal = (value < 0) ? -value : value;
	//
	//vInt1 = vVal;                  // Get the integer (678).
	//vFrac = vVal - vInt1;      // Get fraction (0.0123).
	//vInt2 = trunc(vFrac * 10000);  // Turn into integer (123).
	//
	//sprintf (floatStr, "Byte:  %d.%04d \n", vInt1, vInt2);
	//at64c1_transmit_str(floatStr);	
//}

void at64c1_transmit_byte(uint8_t value) {
	char floatStr[100];
	sprintf (floatStr, "\r\nByte:  "BYTE_TO_BINARY_PATTERN, BYTE_TO_BINARY(value));
	at64c1_transmit_str(floatStr);
	
}