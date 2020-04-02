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
	MCUCR &= ~(1<<SPIPS);
	//DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK)
	
	//set interrupt, enable SPI, MSB mode, set as master, low is idle, read on leading edge, Set to speed(fosc/64) SPR0=1, SPR1=0
	//    (Reset Interupt)|(Enable SPI)(MSB transmitted first)|(Make Master[MSTR])|(SCL is low on idle)|(Leading edge sample)|(Set Clocl rate f_osc/16[SPR0=1|SPR1=0])
	SPCR = (0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(1<<SPR0)|(0<<SPR1);
	//SPCR = 0b01010001;
	SPSR |= (1<<SPI2X);	// Double SPI Speed bit
	SPDR = 0x00;		// Ensure data register has nothing in it
	
	/* Set MOSI and SCK output, all others input */
	DDRC = (1<<PIN_SS);	
}

uint8_t SPI_send_byte(uint8_t c)
{
	SPDR = c;					//write data to the transmission register. Writing to this initiates transmission.
	while(!(SPSR & (1<<SPIF)));
	return SPDR;				//return any data that was shifted into this register upon transmission.
}

// FULLY WORKING
void spi_transfer_buffer(uint8_t *buf, uint8_t count)
{
	if (count == 0) return;
	for(uint8_t i = 0; i < count; i++) {
		uint8_t out = *(buf + i);
		SPDR = out;
		while(!(SPSR & (1<<SPIF)));
		*(buf + i) = SPDR;
	}
}



void spi_disable(void) {
	SPDR = 0;
}


unsigned char spiMasterTRANSMIT(unsigned char data) {
    /* Start transmission */
    SPDR = data;
    /* Wait for transmission complete */
    while(!(SPSR & (1<<SPIF)));
    /* SPDR must be stored as quickly
        as possible (ref. ATMegaX ds) */
    return SPDR;
} 

void spiMasterChipSelect(unsigned char state) {
    /* What the user wants? (remember that the CS signal is inverted) */
    if(state) {
        /* Upper the CS pin */
        SET_BIT(PORT_SPI, PIN_SS);        
    } else {
        /* Lower the CS pin */
        CLEAR_BIT(PORT_SPI, PIN_SS);        
    }
}

// Pointer to function which handle change on INT pin handler 
void (*int_handler)(void);

// Initialization of hardware ext. interrupts \param *handler pointer to a function which handle occured interrupt. * \return nothing
void extInterruptINIT(void (*handler)(void)) {
    // Set function pointer 
    int_handler = handler;
    // TODO: Initialize external interrupt on pin INT0 on failing edge 
    //MCUCR |= (1 << ISC01);
    //GICR |= (1 << INT0); 
}
// System interrupt handler 
SIGNAL(INT0_vect) {
    int_handler();
}