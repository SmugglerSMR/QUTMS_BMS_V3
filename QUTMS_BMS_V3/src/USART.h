/*
 * USART.h
 *
 * Created: 05.12.2019 8:41:46
 *  Author: User
 */ 


#ifndef USART_H_
#define USART_H_

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <avr/sfr_defs.h>
#include "stdio.h"

#define BAUD 115200                        // Baud rate


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

void USART_1_init(void);
int at64c1_transmit (unsigned char byte_data);
void at64c1_transmit_str(unsigned char* str);
void at64c1_transmit_byte(uint8_t value);


#endif /* USART_H_ */