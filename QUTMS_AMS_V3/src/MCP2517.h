/*****************************************************************************
* @file    firmware/QUTMS_HVBoard_Firmware/MCP2517.h
* @author  Zoe Goodward
* @version V1.0.0
* @date    21/11/2019 5:50:22 PM
* @brief   This file..
*****************************************************************************/

#ifndef MCP2517_H_
#define MCP2517_H_

#include "MCP2517_defines.h"

// Transmit Channels
#define TX_FIFO MCP2517_FIFO_CH1
// Receive Channels
#define RX_FIFO MCP2517_FIFO_CH2

#define MAX_MSG_SIZE 21

// CAN IDs
#define CAN_ID_INV	        0x0C000000 /**< CAN Bus Identifier for Inverters */
#define	CAN_ID_AMU	        0x0A000000 /**< CAN Bus Identifier for AMU */
#define CAN_ID_PDM	        0x09000001 /**< CAN Bus Identifier for PDM */
#define CAN_ID_WHEEL	    0x00400000 /**< CAN Bus Identifier for Wheel */

/* CAN CS Ports */
#define CAN_CS_PORT		PORTA
#define CAN_CS_A		PINA0
#define CAN_CS_B		PINA1


static uint8_t CAN_CS=PINA0;

void MCP2517_init();

void enterWordInBufferAtIndex (const uint32_t value, uint8_t buff[], const uint8_t index);
uint32_t wordFromBufferAtIndex (uint8_t buff[], const uint8_t index);

void MCP2517_writeReg8(const uint16_t regAddr, const uint8_t value);
void MCP2517_writeReg32(const uint16_t regAddr, const uint32_t value);
uint8_t MCP2517_readReg8(const uint16_t regAddr);
uint32_t MCP2517_readReg32(const uint16_t regAddr);

void MCP2517_assertCS();
void MCP2517_deassertCS();

void MCP2517_reset();
void MCP2517_setMode(MCP2517_OPERATION_MODE opMode);
uint8_t MCP2517_getMode();
void MCP2517_testRAM();

uint8_t MCP2517_receiveFifoStatus(MCP2517_FIFO_CHANNEL channel, MCP2517_RX_FIFO_STATUS *flags);
uint8_t MCP2517_transmitFifoStatus(MCP2517_FIFO_CHANNEL channel, MCP2517_TX_FIFO_STATUS *flags);

void MCP2517_recieveMessage(uint32_t *receiveID, uint8_t *numDataBytes, uint8_t *data);
void MCP2517_readMsgReceive(uint32_t *receiveID, uint8_t *numDataBytes, uint8_t *data, MCP2517_RX_MSG_OBJ *rxObj);

uint8_t MCP2517_transmitMessage(uint32_t canMessageID, uint8_t numDataBytes, uint8_t *messageData);
void MCP2517_loadMsgTXFifo(MCP2517_TX_MSG_OBJ *txObj, uint8_t *payload, uint8_t numDataBytes);

#endif /* MCP2517_H_ */