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
#include "MCP2517_defines.h"
#include "MCP2517_reg.h"

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

// Transmit Channels
#define MCP2517_TX_FIFO MCP2517_FIFO_CH1
// Receive Channels
#define MCP2517_RX_FIFO MCP2517_FIFO_CH2

#define MCP2517_MAX_MSG_SIZE 21

// CAN IDs
#define CAN_ID_INV	        0x0C000000 /**< CAN Bus Identifier for Inverters */
#define	CAN_ID_AMS	        0x0A000000 /**< CAN Bus Identifier for AMU */
#define	CAN_ID_BMS	        0x0A000001 /**< CAN Bus Identifier for BMS */
#define CAN_ID_PDM	        0x09000001 /**< CAN Bus Identifier for PDM */
#define CAN_ID_WHEEL	    0x00400000 /**< CAN Bus Identifier for Wheel */


void MCP2517FD_Init_Registers(void);
void MCP2517_init(void);

void enterWordInBufferAtIndex (const uint32_t value, uint8_t buff[], const uint8_t index);
uint32_t wordFromBufferAtIndex (uint8_t buff[], const uint8_t index);

void MCP2517_writeReg8(const uint16_t regAddr, const uint8_t value);
void MCP2517_writeReg32(const uint16_t regAddr, const uint32_t value);
uint8_t MCP2517_readReg8(const uint16_t regAddr);
uint32_t MCP2517_readReg32(const uint16_t regAddr);

void MCP2517_assertCS(void);
void MCP2517_deassertCS(void);

void MCP2517_reset(void);
void MCP2517_setMode(MCP2517_OPERATION_MODE opMode);
uint8_t MCP2517_getMode(void);
void MCP2517_testRAM(void);

uint8_t MCP2517_receiveFifoStatus(MCP2517_FIFO_CHANNEL channel, MCP2517_RX_FIFO_STATUS *flags);
uint8_t MCP2517_transmitFifoStatus(MCP2517_FIFO_CHANNEL channel, MCP2517_TX_FIFO_STATUS *flags);

void MCP2517_recieveMessage(uint32_t *receiveID, uint8_t *numDataBytes, uint8_t *data);
void MCP2517_readMsgReceive(uint32_t *receiveID, uint8_t *numDataBytes, uint8_t *data, MCP2517_RX_MSG_OBJ *rxObj);

uint8_t MCP2517_transmitMessage(uint32_t canMessageID, uint8_t numDataBytes, uint8_t *messageData);
void MCP2517_loadMsgTXFifo(MCP2517_TX_MSG_OBJ *txObj, uint8_t *payload, uint8_t numDataBytes);

static uint8_t BMS_BOARD_DATA[5] = {0};
#endif /* MCP2517FD_H_ */