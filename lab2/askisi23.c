#define F_CPU 16000000UL
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

volatile uint8_t counter_time = 0;
volatile uint16_t first_int = 0;

void spinthirismos (void){
	EIFR = (1 << INTF1);
	_delay_ms(5);
	
	if (EIFR & (1 << INTF1)){
		spinthirismos();
	}
}

ISR (INT1_vect){
	spinthirismos();

	counter_time = 0;
	first_int = 1;
	
}

int main(void)
{
	// Configure interruption
	EICRA = (1 << ISC11) | (1 << ISC10);
	EIMSK = (1 << INT1);
	sei();
	
	// Configure i/o
	DDRD = 0x00;
	DDRB = 0xFF;
	PORTB = 0x00;
	
	counter_time = 0;
    while(1)
    {
		if (first_int){
			first_int = 0;
			
			if(PORTB == 0x00){
				PORTB = 0b00001000;
				counter_time = 0;
				
				while(counter_time < 8){
					_delay_ms(500);
					counter_time += 1;
					
					if(first_int){
						first_int = 0;
						PORTB =0xFF;
						_delay_ms(1000);
						PORTB = 0b00001000;
						counter_time = 2;
					}
				}
				PORTB = 0b00000000;
			}
		}
	}
}