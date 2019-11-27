/*
 * SPI.c
 *
 * Created: 25.04.2019 19:07:51
 *  Author: julius
 */ 
#include "SPI.h"

void SPI_init(void)
{
	// When Addid SPIPS as 0 - SPi signals directed to MISO, MOSI, SCK and SS
	// Whed SPIPS as 1 - Alternate SPi pins, MISO_A, MOSI_A, SCK_A, SS_A
	//MCUCR &= ~(1<<SPIPS);
	//DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK)
	
	SPCR =	 (0<<SPIE)	//(Reset Interupt)
			|(1<<SPE)	//(Enable SPI)
			|(0<<DORD)	//(MSB transmitted first)
			|(1<<MSTR)	//(Make Master[MSTR])
			|(0<<CPOL)	//(SCL is low on idle)
			|(1<<CPHA)	//(Leading edge sample)
			|(1<<SPR0)	//(Set Clocl rate f_osc/16[SPR0=1|SPR1=0])
			|(0<<SPR1);
	//set interrupt, enable SPI, MSB mode, set as master, low is idle, read on leading edge, Set to speed(fosc/4) SPR0=1, SPR1=0
	SPSR = (1<<SPI2X);	//set whether we want 2x speed or not (1=2x speed).
	SPDR = 0x00;		//ensure data register has nothing in it
}

uint8_t SPI_send_byte(uint8_t c)
{
	SPDR = c;					//write data to the transmission register. Writing to this initiates transmission.
	while(!(SPSR & (1<<SPIF)));
	return SPDR;				//return any data that was shifted into this register upon transmission.
}

void SPI_MasterTransmit(char cData) {
	/*Start transmission*/
	SPDR = cData;
	/*Wait for transmission complete*/
	while(!(SPSR & (1<<SPIF)));
}