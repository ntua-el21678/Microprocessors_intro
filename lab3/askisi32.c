#define F_CPU 16000000UL
#include "avr/io.h"
#include <util/delay.h>
#include <avr/pgmspace.h>

//twra einai antithetis logikis, mpori na xreiazete allagi
#define PB4_pressed (PINB & 0b11101111)
#define PB5_pressed (PINB & 0b11011111)

//create table to store duty cycle values
volatile uint8_t levels[17] = { 5,20,36,51,66,82,97,112,128,143,158,173,189,204,219,235,250};
volatile uint8_t dc_value;
volatile uint8_t index;
volatile uint8_t timer = 0;
volatile uint8_t sum_counter = 0;
volatile uint16_t result = 0;
volatile uint16_t sum = 0;
volatile uint16_t adc_result = 0;


int main(){
	//set TMR1A in fast PWM 8 bit
	//prescale = 1024 to accomplish 62.5kHz
	TCCR1A = (1 << WGM10) | (1 << COM1A1);
	TCCR1B = (1 << CS12) | (1 << CS10) | (1 << WGM12);

	//set PB1 as output and PB4 & PB5 as input
	DDRB = 0b11001111;

	//set PORTD as output
	DDRD = 0b11111111;
	
	PORTD = 0b00000000;

	//set dc_value to 50%
	index = 8;
	dc_value = levels[index];
	OCR1AL = dc_value;

	//adc init
	ADMUX = (1 << REFS0) | (1 << MUX0);       // ADC1 (PC1), AVcc reference
	ADCSRA = (1 << ADEN) | (1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0); //128 division => 125kHz
	//keep ADLAR = 0 to have 10 bits result in adch:adcl
	
	//create table to keep last 16 measurements
	volatile uint16_t arr[16];
	for (int i=0; i < 16; i++)
		arr[i] = 0;

	while(1){
		if (PB4_pressed){
			//while(PB4_pressed);
			if (index < 16)
				index++;
			dc_value = levels[index];
			OCR1AL = dc_value;
			_delay_ms(10);
		}
		if (PB5_pressed){
			//while(PB5_pressed);
			if (index > 0)
				index--;
			dc_value = levels[index];
			OCR1AL = dc_value;
			_delay_ms(10);
		}
		
		//no button pressed = 8 cycles -> 0.50μs
		//one button pressed = 22 cycles -> 1.38μs
		timer++;
		if (timer > 20){
			timer = 0;
			sum_counter ++;
			
			ADCSRA |= (1 << ADSC);
			while (ADCSRA & (1 << ADSC));   //wait for adc to complete
			adc_result = ADC;
			
			uint8_t idx = sum_counter % 16;

			if (sum_counter < 16){
				arr[idx] = adc_result;
				sum += adc_result;
			}
			else {
				sum += adc_result - arr[idx];
				arr[idx] = adc_result;
				
				result = sum >> 4;
				if (result <= 200)
					PORTD = 0b00000001;
				else if (result <= 400)
					PORTD = 0b00000010;
				else if (result <= 600)
					PORTD = 0b00000100;
				else if (result <= 800)
					PORTD = 0b00001000;
				else
					PORTD = 0b00010000;
			}
		}
	}

}