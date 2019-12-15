/*
 * USART.h
 *
 * Created: 05.12.2019 8:41:46
 *  Author: User
 */ 


#ifndef USART_H_
#define USART_H_

#define F_CPU 16000000UL

void USART_1_init();
int at64c1_transmit (unsigned char byte_data);
void at64c1_transmit_str(unsigned char* str);


#endif /* USART_H_ */