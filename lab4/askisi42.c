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

// Function Prototypes
void lcd_init(void);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_clear(void);
void write_2_nibbles(uint8_t data);
void calc_adc(void);

// Global variable for ADC result
volatile uint16_t adc_result = 0;

int main(void) {
    // Initialize LCD
    lcd_init();
    _delay_ms(100);
    
    // Initialize ADC
    // Set ADC reference to AVCC (5V) and select ADC1 (PC1)
    ADMUX = (1 << REFS0) | (1 << MUX0) | (1 << MUX1);    
    // Enable ADC, Enable ADC interrupt, Set prescaler to 128 (125kHz @ 16MHz)
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    
    // Main loop
    while (1) {
        // Start ADC conversion
        ADCSRA |= (1 << ADSC);
		// wait for adc to finish
        while (ADCSRA & (1 << ADSC));
		calc_adc();
		
        // Wait 500ms between conversions
        _delay_ms(1000);
    }
    
    return 0;
}


void calc_adc() {
    // Read ADC result (10-bit)
    uint16_t adc_value = ADC;  // ADCL and ADCH combined
    
    // Calculate voltage: (adc * 500) / 1024
    uint32_t temp = (uint32_t)adc_value * 500;
    temp = temp >> 10;  // Divide by 1024 (right shift 10 bits)
    
    // Now temp is in centivolts (0-499 for 0.00V-4.99V)
    
    // Display on LCD
    lcd_clear();
    
    // Set cursor to start position
    lcd_command(0x80);  // Row 0, column 0
    
    // Extract whole volts and decimal parts
    uint8_t volts = temp / 100;           // Whole volts (0-4)
    uint8_t cents = temp % 100;           // Centivolts (0-99)
    uint8_t tens = cents / 10;            // Tens digit
    uint8_t ones = cents % 10;            // Ones digit
    
    // Display voltage in format "x.yy V"
    lcd_data('0' + volts);                // Whole volt
    lcd_data('.');                        // Decimal point
    lcd_data('0' + tens);                 // First decimal
    lcd_data('0' + ones);                 // Second decimal
    lcd_data(' ');
    lcd_data('V');
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
