#define F_CPU 16000000UL
#include "avr/io.h"
#include <util/delay.h>
#include <avr/pgmspace.h>

//twra einai antithetis logikis, mpori na xreiazete allagi
#define PB4_pressed (!(PINB & (1 << PB4)))
#define PB5_pressed (!(PINB & (1 << PB5)))
#define PD0_pressed (!(PIND & (1 << PD0)))
#define PD1_pressed (!(PIND & (1 << PD1)))

//create table to store duty cycle values
volatile uint8_t levels[17];
volatile uint8_t dc_value;
volatile uint8_t index;
volatile uint8_t mode1_value = 0;
volatile uint8_t mode2_value = 0;

void save_data(){
	volatile uint8_t temp = 2;
	volatile uint8_t step = 15;
	volatile uint8_t i = 0;

	while(temp < 255){
		levels[i] = temp;
		temp += step;
		i++;
	}
}

void mode1(){
	//while(1){
		OCR1AL = mode1_value;
		
		if (PB4_pressed){
			if (index < 16)
			index++;
			dc_value = levels[index];
			OCR1AL = dc_value;
			_delay_ms(10);
		}
		if (PB5_pressed){
			if (index > 0)
			index--;
			dc_value = levels[index];
			OCR1AL = dc_value;
			_delay_ms(10);
		}
		
		mode1_value = OCR1AL;
		return;
	//}
}

void mode2(){
	OCR1AL = mode2_value;
	
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	dc_value = ADCH;
	OCR1AL = dc_value;
	
	mode2_value = OCR1AL;
	
	return;
}

int main(){

	//set TMR1A in fast PWM 8 bit
	//prescale = 1024 to accomplish 62.5kHz
	TCCR1A = (1 << WGM10) | (1 << COM1A1);
	TCCR1B = (1 << CS12) | (1 << CS10) | (1 << WGM12);

	//set PB1 as output and PB4/PB5 as input
	DDRB = 0b11001111;

	//set PORTD as input
	DDRD = 0b11111100;

	//save duty cycle levels in SRAM
	save_data();

	//set dc_value to 50%
	index = 8;
	dc_value = levels[index];

	//adc init
	ADMUX = (1 << REFS0) | (1 << ADLAR);     // ADC0 (PC0), AVcc reference
	ADCSRA = (1 << ADEN) | (1 << ADPS2 | 1 << ADPS1 | 1 << ADPS0); //128 division => 125kHz

	while(1){
		while (PD0_pressed)
			mode1();
		while (PD1_pressed)
			mode2();
	}

}

