/*
 * MCP2517FD.c
 *
 * Created: 22/11/2019 1:33:46 PM
 *  Author: sadykov
 */ 
#include "MCP2517FD.h"

void MCP2517FD_reg_write() {
	// unset to start transmission. Critical Section.	
	WRITE_BIT(MCP2517FD_PORT_CS, MCP2517FD_PIN_CS, LOW);
	SPI_send_byte(0x00);
	// Set back.
	WRITE_BIT(MCP2517FD_PORT_CS, MCP2517FD_PIN_CS, HIGH);
	// Make sure that extra time is needed
	_delay_us(80);
}