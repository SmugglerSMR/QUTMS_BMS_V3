/*
 * SPI.h
 *
 * Created: 25.04.2019 19:10:04
 *  Author: julius
 */ 


#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>

void SPI_init(void);
uint8_t SPI_send_byte(uint8_t c);
void SPI_MasterTransmit(char cData);

#endif /* SPI_H_ */