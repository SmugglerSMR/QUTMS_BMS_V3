/*
 * MAX14920.c
 *
 * Created: 19/11/2019 3:48:20 PM
 *  Author: sadykov
 */ 
#include "MAX14920.h"
#include "macros.h"
#include "SPI.h"
#include "ADC.h"


static const int cellTable[MAX14920_CELL_NUMBER] = {
	0b0000, 0b1000, 0b0100, 0b1100,	// Cells 1, 2, 3, 4
	0b0010, 0b1010, 0b0110, 0b1110,	// Cells 5, 6, 7, 8
	0b0001, 0b1001, 0b0101, 0b1101	// Cells 9,10,11,12
};

void MAX14920_Init_Registers(void) {
	DDRB |= ((1<<MAX14920_PIN_SCK)|
			(1<<MAX14920_PIN_MOSI)|
			~(1<<MAX14920_PIN_MISO)); // MISO as input
	DDRC |= ((1<<MAX14920_PIN_SAMPL)|
			 (1<<MAX14920_PIN_CS));
	DDRD |= ((1<<MAX14920_PIN_EN)|
			(1<<MAX14920_PIN_SS));
	
	// Set SS as high to disable transmission.
	WRITE_BIT(MAX14920_PORT_CS, MAX14920_PIN_CS, HIGH);
	// TODO: Try make MISO and MOSO low
	
	// Allow sampling through SPI messages.
	WRITE_BIT(MAX14920_PORT_SAMPL, MAX14920_PIN_SAMPL, HIGH);
	
	// Shutdown and reset SPI.
	WRITE_BIT(MAX14920_PORT_EN, MAX14920_PIN_EN, LOW);
}
void MAX14920_Clear_SPI_messages(void) {
	// Disable Cell balancers and Move to sampling phase
	MAX14920_SPI_message.spiBalanceC01_C08 = 0x00;
	MAX14920_SPI_message.spiBalanceC09_C16 = 0x00;
	MAX14920_SPI_message.spiEnableCellSelect = 0;
	MAX14920_SPI_message.spiCell4bit = 0b0000;
	MAX14920_SPI_message.spiSMPLB = 0;
	MAX14920_SPI_message.spiDIAG = 0;
	MAX14920_SPI_message.spiLOPW = 0;
	
	// Clear output messages
	MAX14920_SPI_output.spiCellStatusC01_C08 = 0x00;
	MAX14920_SPI_output.spiCellStatusC09_C16 = 0x00;
	MAX14920_SPI_output.spiChipStatus = 0b10101011;
}
void MAX14920_reg_write() {		
	// unset to start transmission. Critical Section.
	uint8_t spiStatus = 0;
	spiStatus |= (
		MAX14920_SPI_message.spiEnableCellSelect<<7 |
		MAX14920_SPI_message.spiCell4bit<<3 |
		MAX14920_SPI_message.spiSMPLB<<2 |
		MAX14920_SPI_message.spiDIAG<<1 |
		MAX14920_SPI_message.spiLOPW
	);	
	WRITE_BIT(MAX14920_PORT_CS, MAX14920_PIN_CS, LOW);
	MAX14920_SPI_output.spiCellStatusC01_C08 = SPI_send_byte(MAX14920_SPI_message.spiBalanceC01_C08);	
	MAX14920_SPI_output.spiCellStatusC09_C16 = SPI_send_byte(MAX14920_SPI_message.spiBalanceC09_C16);	
	MAX14920_SPI_output.spiChipStatus = SPI_send_byte(spiStatus);
	// Set back.
	WRITE_BIT(MAX14920_PORT_CS, MAX14920_PIN_CS, HIGH);
	// Make sure that extra time is needed
	_delay_us(80);	
}

void MAX14920_Enable(void) {
	uint8_t status = 0b00000010;
	//PORTC |= (1<<PINC6); // Set SAMPL high to track voltage at CV	
	// SET EN High to enable device shutdown mode. 
	//	It resets the SPI register, make sure it's on
	SET_BIT(MAX14920_PORT_EN, MAX14920_PIN_EN);
	_delay_ms(1+8);	//+8for calibration
	// fIRST 8 BIT NEVER GETS SEND
	MAX14920_SPI_message.spiSMPLB = 1;
	MAX14920_reg_write();
	
	while(status&(MAX14920_SPI_output.spiChipStatus)) {
		MAX14920_reg_write();
		// Wait a bit before try again.
		Toggle_LED(5, 100, 1);
	}
	PORTB ^= 0b00001000;	// Indicate success of MAX14920 LED4
}

void MAX14920_OffsetCallibration(void){
	// Set SAMPL high to start calibration
	WRITE_BIT(MAX14920_PORT_SAMPL, MAX14920_PIN_SAMPL, HIGH);
	
	//MAX14920_reg_write(0x00,0x00,0b01000000);
	_delay_ms(8);
	// Set SAMPL back to low to as not necessary.
	WRITE_BIT(MAX14920_PORT_SAMPL, MAX14920_PIN_SAMPL, LOW);
}

void MAX14920_EnableHoldPhase(bool sample) {
	if(sample) {
		// Move to Hold phase all cell voltages are simultaneously sampled on their 
		//	associated capacitors.		
		//WRITE_BIT(MAX14920_SPI_message.spiControl, MAX14920_SMPLB_bit, 1);
		MAX14920_SPI_message.spiSMPLB = 1;
	} else {
		// Move to sampling phase
		// AOUT suppose to be equal Vp/12 = For us 2.5V		
		//WRITE_BIT(MAX14920_SPI_message.spiControl, MAX14920_SMPLB_bit, 0);
		MAX14920_SPI_message.spiSMPLB = 0;
	}
	_delay_us(50);
}
uint16_t MAX14920_ReadData(void) {
	// In sample phase ADC = Vp/12, Vp = 30.0, 
	uint16_t voltage = 0; 
	//Getting ADC value
	uint16_t ADC_v = adc_read(6);
	SPI_send_byte(0b00001111);
	SPI_send_byte((uint8_t)(ADC_v>>8));
	SPI_send_byte((uint8_t)ADC_v);
	//voltage = ADC_v*1000 * 5.0 / 1023;
	//_delay_us(100);
	//SPI_send_byte((uint8_t)(voltage>>8));
	//SPI_send_byte((uint8_t)voltage);
	
	return ADC_v;
}
//Need a structure, to keep what balancing command has been sent already, as a global variable.
uint16_t MAX14920_ReadCellVoltage(int cellN) {
	
	// Disable Sampler??
	//SET_BIT(MAX14920_PORT_CS, PINC6);	
	MAX14920_EnableHoldPhase(true);
	if(cellN == 0) {		// Read overall voltage		
		MAX14920_SPI_message.spiEnableCellSelect = 0;
		MAX14920_SPI_message.spiCell4bit = 0011;
		MAX14920_reg_write();
		_delay_us(60);
		
	} else if(cellN > 0) {	// Read cell by cell
		MAX14920_SPI_message.spiEnableCellSelect = 1;
		MAX14920_SPI_message.spiCell4bit = cellTable[cellN-1];
		MAX14920_reg_write();
		_delay_us(10);		
	}						// No negative cell numbers
	else return 0.0;
	
	MAX14920_EnableHoldPhase(false);
	MAX14920_reg_write();
	return (uint16_t)(MAX14920_ReadData()*4.921/1023*1000);
}

void MAX14920_ReadAllCellsVoltage(void) {	
	//CellVoltages[10] = {0};
	OveralVoltage = 0;
	AverageCellVoltage = 0;
	MaxCellVoltage = 0;
	MinCellVoltage = 0;
	
	for (int cellN = 1; cellN<=MAX14920_CELL_NUMBER-2; cellN++) {
		CellVoltages[cellN-1] = MAX14920_ReadCellVoltage(cellN)*100;
		AverageCellVoltage +=CellVoltages[cellN-1];
		if(CellVoltages[cellN-1] > MaxCellVoltage)
			MaxCellVoltage = CellVoltages[cellN-1];
		if(CellVoltages[cellN-1] < MinCellVoltage)
			MinCellVoltage = CellVoltages[cellN-1];
	}
	AverageCellVoltage /= MAX14920_CELL_NUMBER-2;
}

#define BALANCING_THRESHOLD	112
void MAX14920_EnableLoadBalancer(bool enable) {
	//average voltage out of others
	float difference = 0.0;
	
	for(int i = 0; i<10;i++) {
		for(int j = 0; j<10;j++) {
			if(abs(CellVoltages[i] - CellVoltages[j]) > difference) {
				difference = abs(CellVoltages[i] - CellVoltages[j]);
			}
			// TODO: Recheck indexation later
			if(difference > BALANCING_THRESHOLD && i <= 8) {
				MAX14920_SPI_message.spiBalanceC01_C08 |= (1<<(7-i));
				difference = 0.0;
			} else if (difference > BALANCING_THRESHOLD && i > 8) {
				MAX14920_SPI_message.spiBalanceC09_C16 |= (1<<(7-i));
				difference = 0.0;
			}
			if(i < 8) {
				MAX14920_SPI_message.spiBalanceC01_C08 |= ~(1<<(7-i));
			} else {
				MAX14920_SPI_message.spiBalanceC09_C16 |= ~(1<<(7-i));
			}
		}		
	}
}

void MAX14920_PerformDiagnosticsFirst(void) {
	// Make sure that sampling is running
	MAX14920_EnableHoldPhase(false);
	MAX14920_SPI_message.spiDIAG = 1;
	MAX14920_reg_write();
	_delay_ms(550);
	//MAX14920_SPI_message.spiDIAG = 0;	
	MAX14920_EnableHoldPhase(true);
	MAX14920_reg_write();
	if(MAX14920_SPI_output.spiCellStatusC01_C08 ||
		MAX14920_SPI_output.spiCellStatusC09_C16) {
			// Report Failure
			SPI_send_byte(0b01010101);
			PORTB ^= 0b00010000;
		}
	MAX14920_ReadAllCellsVoltage();
	SPI_send_byte(0b10000001);
	for(int i=0;i<MAX14920_CELL_NUMBER-1;i++)
		if(CellVoltages[i] < 122)
			SPI_send_byte(i);
	MAX14920_SPI_message.spiDIAG = 0;
	MAX14920_EnableHoldPhase(true);
	MAX14920_reg_write();
}

void MAX14920_PerformDiagnosticsSecond(void) {
	// Make sure that sampling is running	
	for(int i=0; i<MAX14920_CELL_NUMBER-2; i++) {
		MAX14920_EnableHoldPhase(false);
		if(i < 8) {
			MAX14920_SPI_message.spiBalanceC01_C08 = 1<<(7-i);
			MAX14920_SPI_message.spiBalanceC09_C16 = 0; 
		} else {
			MAX14920_SPI_message.spiBalanceC01_C08 = 0;
			MAX14920_SPI_message.spiBalanceC09_C16 = 1<<(7-(i-9));
		}		
		MAX14920_reg_write();
		_delay_us(300);//R_BAL*C_SAMPLE 3K*1nF
		if(MAX14920_ReadCellVoltage(i+1) == 0){
			SPI_send_byte(0b01010101);
			PORTB ^= 0b00010000;
			SPI_send_byte(i);
		}
	}
	
}