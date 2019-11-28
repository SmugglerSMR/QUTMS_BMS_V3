/*****************************************************************************
* @file    firmware/QUTMS_SteeringWheel_Firmware/input.h
* @author  Zoe Goodward
* @version V1.0.0
* @date    2/08/2019 1:20:22 PM
* @brief   This file defines the outputs and inputs of the pins of the MCU
*****************************************************************************/

#ifndef INPUT_H_
#define INPUT_H_

/*	Pinout
	RX			PD0
	TX			PD1
	
*/

//#define OLED_RESET_HIGH	PORTB |= 0b00000001
//#define OLED_RESET_LOW PORTB &= ~0b00000001
//#define OLED_DC_DATA PORTD |= 0b00001000
//#define OLED_DC_INSTRUCTION PORTD &= ~0b00001000
//#define SPI_CLOCK_HIGH PORTB |= 0b10000000
//#define SPI_CLOCK_LOW PORTB &= ~0b10000000
//#define SPI_DATA_HIGH PORTB |= 0b00100000
//#define SPI_DATA_LOW PORTB &= ~0b00100000


/* CAN */
#define CAN_CS_PORT		PORTB
#define CAN_CS		PINB3
#define CAN_INT		PINB0

/* LEDS */
//#define LED_A		PIND6
//#define LED_B		PIND7

//#define LED_A_ON	PORTD |= 0b01000000
//#define LED_A_OFF	PORTD &= ~0b01000000
//#define LED_B_ON	PORTD |= 0b10000000
//#define LED_B_OFF	PORTD &= ~0b10000000

#endif /* INPUT_H_ */