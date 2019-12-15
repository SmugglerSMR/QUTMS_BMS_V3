/*
 * USART.c
 *
 * Created: 05.12.2019 8:39:52
 *  Author: User
 */ 
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <avr/sfr_defs.h>

#define BAUD 9600                        // Baud rate


// ********************************* Initialisation USART *********************************

void USART_1_init() {
	LINCR = (1 << LSWRES);
	LINBRRH = (((F_CPU/BAUD)/16)-1)>>8;
	LINBRRL = (((F_CPU/BAUD)/16)-1);
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