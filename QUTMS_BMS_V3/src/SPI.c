/*
 * SPI.c
 *
 * Created: 25.04.2019 19:07:51
 *  Author: julius
 */ 
#include "SPI.h"

void SPI_init()
{
	// When Addid SPIPS as 0 - SPi signals directed to MISO, MOSI, SCK and SS
	// Whed SPIPS as 1 - Alternate SPi pins, MISO_A, MOSI_A, SCK_A, SS_A
	MCUCR &= ~(1<<SPIPS);
	
	//set interrupt, enable SPI, MSB mode, set as master, low is idle, read on leading edge, Set to speed(fosc/64) SPR0=1, SPR1=0
	//    (Reset Interupt)|(Enable SPI)(MSB transmitted first)|(Make Master[MSTR])|(SCL is low on idle)|(Leading edge sample)|(Set Clocl rate f_osc/16[SPR0=1|SPR1=0])
	SPCR = (0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR0)|(0<<SPR1);
	SPSR = (1<<SPI2X);	// Double SPI Speed bit
	SPDR = 0x00;		// Ensure data register has nothing in it
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