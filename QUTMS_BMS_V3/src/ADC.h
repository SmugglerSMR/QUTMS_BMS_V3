/*
 * ADC.h
 *
 * Created: 19/11/2019 3:30:27 PM
 *  Author: sadykov
 */ 


#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>

void ADC_init(void);
uint16_t ADC6_read(void);
uint16_t adc_read(uint8_t channel);
int adc2_read(uint8_t channel);


#endif /* ADC_H_ */