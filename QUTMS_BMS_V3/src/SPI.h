/*
 * SPI.h
 *
 * Created: 25.04.2019 19:10:04
 *  Author: julius
 */ 


#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>

struct MAX14920_SPI_SDI {
	uint8_t spiBalanceC01_C08;
	uint8_t spiBalanceC09_C16;
	uint8_t spiEnableCellSelect;
	uint8_t spiCell4bit;
	uint8_t spiSMPLB;
	uint8_t spiDIAG;
	uint8_t spiLOPW;
	
};

struct MAX14920_SPI_SDO {
	uint8_t spiCellStatusC01_C08;
	uint8_t spiCellStatusC09_C16;
	uint8_t spiChipStatus;
};

static volatile struct MAX14920_SPI_SDI MAX14920_SPI_message;
static volatile struct MAX14920_SPI_SDO MAX14920_SPI_output;

void SPI_init(void);
uint8_t SPI_send_byte(uint8_t c);
void SPI_MasterTransmit(char cData);

#endif /* SPI_H_ */