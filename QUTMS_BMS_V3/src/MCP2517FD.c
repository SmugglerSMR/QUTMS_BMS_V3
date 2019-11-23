/*
 * MCP2517FD.c
 *
 * Created: 22/11/2019 1:33:46 PM
 *  Author: sadykov
 */ 
#include "MCP2517FD.h"

void MCP2517FD_reg_write(uint8_t reg_address, uint8_t reg_value) {
	// unset to start transmission. Critical Section.	
	WRITE_BIT(MCP2517FD_PORT_CS, MCP2517FD_PIN_CS, LOW);
	SPI_send_byte(MCP2515_WRITE);
	SPI_send_byte(reg_address);
	SPI_send_byte(reg_value);
	// Set back.
	WRITE_BIT(MCP2517FD_PORT_CS, MCP2517FD_PIN_CS, HIGH);
	// Make sure that extra time is needed
	_delay_us(80);
}