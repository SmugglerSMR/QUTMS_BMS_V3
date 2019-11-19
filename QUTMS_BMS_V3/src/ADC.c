/*
 * ADC.c
 *
 * Created: 19/11/2019 3:30:13 PM
 *  Author: sadykov
 */ 
#include "ADC.h"
#include "macros.h"

void ADC_init(void) {
	// AVcc with capacitor
	// !!!! If any inaccuaracies accured, choise option without capacitor
	//ADMUX = (1<<REFS0)| (1<<AREFEN);	// With capacitor
	ADMUX = (1<<REFS0); // Without capacitor
	CLEAR_BIT(ADMUX, ADLAR); /* make sure bits are right adjusted */	
	// 16MHz clock/128 prescaler= 125kHz = 0.000008s.
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC6_read(void) {
	// Set to PB5 or ADC6 | REFS0 defines AVcc with no external capacitor connected on the AREF pin
	ADMUX = (1<<REFS0)| (1<<ADLAR) | (1<<MUX2) | (1<<MUX1);
	
	// Start single conversion by setting ADSC bit in ADCSRA
	ADCSRA |= (1 << ADSC);
	while ( ADCSRA & (1 << ADSC) ) {}
	return ADC;
}

/*============================================================================
Function:   adc_read()
------------------------------------------------------------------------------
Purpose :   reads an analog input voltage and converts it to a 10-bit digital
			value through successive approximation
Input   :   uint8_t channel - selected analog input channel
Returns :   result - pass the 10 bit ADC number to requesting function
Notes   :
============================================================================*/
uint16_t adc_read(uint8_t channel) 
{   
	/* only change ADMUX bits signalling which channel to use */
    channel = (ADMUX & 0xe0)|(channel & 0x1F); 
	ADMUX = channel;
	
	/* start conversion process */
	SET_BIT(ADCSRA, ADSC);	
	
	/* loop while the conversion is taking place */
	//while(!(CHECK_BIT(ADCSRA, ADIF)));
	while ( ADCSRA & (1 << ADSC) ) {}
	
	uint16_t result = 0;	
	result = ADCL; /* read ADCL, then ADCH --> order is important! */							
	result |= ((3 & ADCH) << 8);
	//--> also not sure if this code is correct. other ADC examples return 'ADC' instead. //
	SET_BIT(ADCSRA, ADIF); /* clear 'complete' status */	
	return result;
}