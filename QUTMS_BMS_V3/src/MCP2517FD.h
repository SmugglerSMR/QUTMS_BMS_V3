/*
 * MCP2517FD.h
 *
 * Created: 22/11/2019 1:33:57 PM
 *  Author: sadykov
 */ 


#ifndef MCP2517FD_H_
#define MCP2517FD_H_

#define F_CPU 16000000UL

#include <avr/io.h>
#include <util/delay.h>

#define MCP2517FD_PIN_SCK	PINB7
#define MCP2517FD_PIN_MOSI	PINB1
#define MCP2517FD_PIN_MISO	PINB0
#define MCP2517FD_PIN_SS	PIND3

#define MCP2517FD_PORT_CS		PORTC
#define MCP2517FD_PIN_CS		PINC7		//***

#define MCP2517FD_RESET       0x00
#define MCP2517FD_READ        0x03
#define MCP2517FD_WRITE		0x02		//*** send this, then send address byte, then send data
#define MCP2517FD_READ_CRC    0x0B
#define MCP2517FD_WRITE_CRC   0x0A
#define MCP2517FD_WRITE_SAFE  0x0C

void MCP2517FD_Init_Registers(void);
void MCP2517FD_reg_write(uint8_t reg_address, uint8_t reg_value);

static uint8_t BMS_BOARD_DATA[5] = {0};
#endif /* MCP2517FD_H_ */