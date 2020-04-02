/*
 * SPI.h
 *
 * Created: 25.04.2019 19:10:04
 *  Author: julius
 */ 


#ifndef SPI_H_
#define SPI_H_

#include <avr/io.h>
#include <macros.h>
//////////////////////////////
/*		  CONFIGURE			*/

// SPI CLock Modes
/* SPI MODE 0:	CPOL = 0, CPHA = 0; Leading Edge: Sample (Rising), Trailing Edge: Setup (Falling) */
/* SPI MODE 1:	CPOL = 0, CPHA = 1; Leading Edge: Setup (Rising), Trailing Edge: Sample (Falling) */
/* SPI MODE 2:	CPOL = 1, CPHA = 0; Leading Edge: Sample (Falling), Trailing Edge: Setup (Rising) */
/* SPI MODE 3:	CPOL = 1, CPHA = 1; Leading Edge: Setup (Falling), Trailing Edge: Sample (Rising) */
#define SPI_MODE 0

// Clock Rate
/* SPI CLOCK RATE 0:	SCK Frequency: Fosc/4 */
/* SPI CLOCK RATE 1:	SCK Frequency: Fosc/16 */
/* SPI CLOCK RATE 2:	SCK Frequency: Fosc/64 */
/* SPI CLOCK RATE 3:	SCK Frequency: Fosc/128 */
/* SPI CLOCK RATE 4:	SCK Frequency: Fosc/2 */
/* SPI CLOCK RATE 5:	SCK Frequency: Fosc/8 */
/* SPI CLOCK RATE 6:	SCK Frequency: Fosc/32 */
/* SPI CLOCK RATE 7:	SCK Frequency: Fosc/64 */
#define SPI_CLK_RATE 0

#define SPI_MSTR 1

#define SPI_MSBFIRST 0

#define SPI_INT_ENABLE 0

/*		END OF CONFIG		*/
//////////////////////////////


#ifdef SPI_INT_ENABLE
#if SPI_INT_ENABLE == 0
#define SPI_INTERRUPT	0
#elif SPI_INT_ENABLE == 1
#define SPI_INTERRUPT	1
#endif
#else
#error SPI: SPIE in SPCR not set
#endif

#ifdef SPI_MSBFIRST
#if SPI_MSBFIRST == 0
#define SPI_DATA_ORDER 0
#elif SPI_MSBFIRST == 1
#define SPI_DATA_ORDER 1
#endif
#else
#error SPI: DORD in SPCR not set
#endif


#ifdef SPI_MSTR
#if SPI_MSTR == 0
#define SPI_MSTR_MODE 0
#elif SPI_MSTR == 1
#define SPI_MSTR_MODE 1
#endif
#else
#error SPI: MSTR in SPCR not set
#endif

#ifdef SPI_MODE
#if SPI_MODE == 0
#define SPI_CLK_POLARITY	0
#define SPI_CLK_PHASE		0
#elif SPI_MODE == 1
#define SPI_CLK_POLARITY	0
#define SPI_CLK_PHASE		1
#elif SPI_MODE == 2
#define SPI_CLK_POLARITY	1
#define SPI_CLK_PHASE		0
#elif SPI_MODE == 3
#define SPI_CLK_POLARITY	1
#define SPI_CLK_PHASE		1
#endif
#else
#error SPI: Data mode (CPOL & CPHA) in SPCR not set
#endif

#ifdef SPI_CLK_RATE
#if SPI_CLK_RATE == 0
#define SPI_CLK_RATE0 0
#define SPI_CLK_RATE1 0
#define DBL_CLK		0
#elif SPI_CLK_RATE == 1
#define SPI_CLK_RATE0 1
#define SPI_CLK_RATE1 0
#define DBL_CLK		0
#elif SPI_CLK_RATE == 2
#define SPI_CLK_RATE0 0
#define SPI_CLK_RATE1 1
#define DBL_CLK		0
#elif SPI_CLK_RATE == 3
#define SPI_CLK_RATE0 1
#define SPI_CLK_RATE1 1
#define DBL_CLK		0
#elif SPI_CLK_RATE == 4
#define SPI_CLK_RATE0 0
#define SPI_CLK_RATE1 0
#define DBL_CLK		1
#elif SPI_CLK_RATE == 5
#define SPI_CLK_RATE0 1
#define SPI_CLK_RATE1 0
#define DBL_CLK		1
#elif SPI_CLK_RATE == 6
#define SPI_CLK_RATE0 0
#define SPI_CLK_RATE1 1
#define DBL_CLK		1
#elif SPI_CLK_RATE == 7
#define SPI_CLK_RATE0 1
#define SPI_CLK_RATE1 1
#define DBL_CLK		1
#endif
#else
#error SPI: Clock rate (SPR0 & SPR1) in SPCR not set
#endif

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

volatile struct MAX14920_SPI_SDI MAX14920_SPI_message;
volatile struct MAX14920_SPI_SDO MAX14920_SPI_output;

/* Platform dependent Registers, Ports and Pins*/
#define DDR_SPI     DDRB    /* Data dir. register for port with SPI */
#define PORT_SPI    PORTC   /* Port with SPI */
#define PIN_MOSI    PINB1     /* MOSI pin on the PORTB_SPI */
#define PIN_MISO    PINB0     /* MISO pin on the PORTB_SPI */
#define PIN_SCK     PINB7     /* SCK pin on the PORTB_SPI */
#define PIN_SS      PINC7     /* SS pin on the PORTB_SPI */

#define PORT_CS     PORTC
#define PIN_CS      PINC7

#define SPI_INT_DDR   DDRD
#define SPI_INT_PORT  PORTD
#define SPI_INT       (1 << (PIND1))

void SPI_init(void);
uint8_t SPI_send_byte(uint8_t c);
void spi_transfer_buffer(uint8_t *buf, uint8_t count);
void spi_disable(void);

/** \brief Transmiting databytes via the SPI
 * 
 * This function is transmitting data via the SPI interface. Input
 * parameter is uns. char array. Data are transmited from the zero
 * index 
 * 
 * \warning This is platform-dependent method!
 * \param data[] Source data array
 * \param length Array length
 * 
 */
unsigned char spiMasterTRANSMIT(unsigned char data);

/** \brief Settings of the CS pin
 * 
 * This function is used for setting of the CS pin. CS signal
 * is inverted, so input 1 (true) means zero on the output.
 * Otherwise is analogically the same.
 * 
 * \warning This is platform-dependent method!
 * \param state Wished state
 */
void spiMasterChipSelect(unsigned char state);

/** Initialization of hardware ext. interrupts
 * \param *handler pointer to a function which handle occured interrupt.
 * \return nothing
 */
//void extInterruptINIT(void (*handler)(void));


#endif /* SPI_H_ */