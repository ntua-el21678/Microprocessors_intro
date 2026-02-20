#define F_CPU 16000000UL  // 16 MHz clock

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

// LCD Pin Definitions (PORTD)
#define LCD_RS PD2
#define LCD_E  PD3
#define LCD_DATA_PORT PORTD
#define LCD_DATA_DDR  DDRD
#define LCD_DATA_PIN  PIND

// global variables
volatile uint16_t vgas = 0;
volatile uint16_t result = 0;
volatile uint8_t flag = 0;

void calc_adc() {
	// Read ADC result (10-bit)
	uint16_t adc_value = ADC;  // ADCL and ADCH combined
	
	// Calculate voltage: (adc * 5.0) / 1024
	float voltage = ((float)adc_value * 5.0) / 1024.0;
	
	// Calculate result: 77.52 * (voltage - 0.1)
	float temp_result = 77.52 * (voltage - 0.1);
	
	// Clamp to positive values
	if (temp_result < 0)
		result = 0; 
	else
		result = (uint16_t)temp_result;
	
	vgas = (uint16_t)(voltage * 100);  // Store voltage in centivolts for display
}

void lcd_print(const char* str) {
	while (*str) {
		lcd_data(*str++);
	}
}

void write_2_nibbles(uint8_t data) {
	uint8_t port_state = LCD_DATA_PIN & 0x0F;  // Preserve lower 4 bits
	
	// Send high nibble
	LCD_DATA_PORT = (data & 0xF0) | port_state;
	LCD_DATA_PORT |= (1 << LCD_E);   // Enable pulse
	_delay_us(1);
	LCD_DATA_PORT &= ~(1 << LCD_E);
	
	// Send low nibble
	LCD_DATA_PORT = ((data << 4) & 0xF0) | port_state;
	LCD_DATA_PORT |= (1 << LCD_E);   // Enable pulse
	_delay_us(1);
	LCD_DATA_PORT &= ~(1 << LCD_E);
}

void lcd_command(uint8_t cmd) {
	LCD_DATA_PORT &= ~(1 << LCD_RS);  // RS = 0 for command
	write_2_nibbles(cmd);
	_delay_us(250);
}

void lcd_data(uint8_t data) {
	LCD_DATA_PORT |= (1 << LCD_RS);   // RS = 1 for data
	write_2_nibbles(data);
	_delay_us(250);
}

void lcd_clear(void) {
	lcd_command(0x01);  // Clear display
	_delay_ms(5);
}

void lcd_init(void) {
	// Set PORTD pins as output for LCD
	LCD_DATA_DDR |= 0xFC;  // PD7-PD2 as outputs
	
	_delay_ms(200);
	
	// Initialize LCD in 4-bit mode
	// Send 0x30 three times
	for (uint8_t i = 0; i < 3; i++) {
		LCD_DATA_PORT = 0x30;
		LCD_DATA_PORT |= (1 << LCD_E);
		_delay_us(1);
		LCD_DATA_PORT &= ~(1 << LCD_E);
		_delay_us(250);
	}
	
	// Switch to 4-bit mode
	LCD_DATA_PORT = 0x20;
	LCD_DATA_PORT |= (1 << LCD_E);
	_delay_us(1);
	LCD_DATA_PORT &= ~(1 << LCD_E);
	_delay_us(250);
	
	// Function set: 4-bit, 2 lines, 5x8 dots
	lcd_command(0x28);
	
	// Display ON, cursor OFF
	lcd_command(0x0C);
	
	// Clear display
	lcd_clear();
	
	// Entry mode: increment cursor, no display shift
	lcd_command(0x06);
}


int main(void){
	//initialize portb as output
	DDRB = 0b11111111;
	PORTB = 0;
	
	// Initialize LCD
	lcd_init();
	_delay_ms(100);
	
	// Initialize ADC
	// Set ADC reference to AVCC (5V) and select ADC3 (PC3)
	ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX1);
	// Enable ADC, Enable ADC interrupt, Set prescaler to 128 (125kHz @ 16MHz)
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	
	
	while(1) {
		// Start ADC conversion
		ADCSRA |= (1 << ADSC);
		// wait for adc to finish
		while (ADCSRA & (1 << ADSC));
		
		// transform adc value into voltage and calculate result
		calc_adc();
		
		// check range of result
		if (result <= 75){
			lcd_clear();
			if (flag == 1)
				lcd_print("CLEAR");
			flag = 0;
			
			PORTB = 0b00000001;
			_delay_ms(1000);
			lcd_clear();
			}
		else if (result <= 150){
			lcd_clear();
			lcd_command(0x80);
			lcd_print("GAS DETECTED");
			PORTB = 0b00000011;
			_delay_ms(800);
			PORTB = 0b00000000;
			flag = 1;
			}
		else if (result <= 220){
			lcd_clear();
			lcd_command(0x80);
			lcd_print("GAS DETECTED");
			PORTB = 0b00000111;
			_delay_ms(800);
			PORTB = 0b00000000;
			flag = 1;
			}
		else if (result <= 280){
			lcd_clear();
			lcd_command(0x80);
			lcd_print("GAS DETECTED");
			PORTB = 0b00001111;
			_delay_ms(800);
			PORTB = 0b00000000;
			flag = 1;
			}
		else if (result <= 350){
			lcd_clear();
			lcd_command(0x80);
			lcd_print("GAS DETECTED");
			PORTB = 0b00011111;
			_delay_ms(800);
			PORTB = 0b00000000;
			flag = 1;
		}
		else{
			lcd_clear();
			lcd_command(0x80);
			lcd_print("GAS DETECTED");
			PORTB = 0b00111111;
			_delay_ms(800);
			PORTB = 0b00000000;
			flag = 1;
		}
		_delay_ms(1000);
	}
}
