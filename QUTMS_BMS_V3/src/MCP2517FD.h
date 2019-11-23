/*
 * MCP2517FD.h
 *
 * Created: 22/11/2019 1:33:57 PM
 *  Author: sadykov
 */ 


#ifndef MCP2517FD_H_
#define MCP2517FD_H_

#define MCP2517FD_PORT_CS		PORTC
#define MCP2517FD_PIN_CS		PINC7		//***

#define MCP2517FD_RESET       0x00
#define MCP2517FD_READ        0x03
#define MCP2517FD_WRITE		0x02		//*** send this, then send address byte, then send data
#define MCP2517FD_READ_CRC    0x0B
#define MCP2517FD_WRITE_CRC   0x0A
#define MCP2517FD_WRITE_SAFE  0x0C

#endif /* MCP2517FD_H_ */