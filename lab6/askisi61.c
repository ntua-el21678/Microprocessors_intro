#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define PCA9555_0_ADDRESS 0x40		//A0=A1=A2=0 by hardware
#define TWI_READ 1					// reading from twi device
#define TWI_WRITE 0					// writing to twi device
#define SCL_CLOCK 100000L			// twi clock in Hz

// LCD Pin Definitions (PORTD)
#define PEX02 2
#define PEX03 3
#define LCD_RS PEX02
#define LCD_E  PEX03
// #define LCD_DATA_PORT PORTD
// #define LCD_DATA_DDR  DDRD
// #define LCD_DATA_PIN  PIND

volatile uint8_t LCD_DATA_PORT;
volatile uint8_t LCD_DATA_DDR;
volatile uint8_t LCD_DATA_PIN;

//Fscl=Fcpu/(16+2*TWBR0_VALUE*PRESCALER_VALUE)
#define TWBR0_VALUE ((F_CPU/SCL_CLOCK)-16)/2

// PCA9555 REGISTERS
typedef enum {
	REG_INPUT_0 = 0,
	REG_INPUT_1 = 1,
	REG_OUTPUT_0 = 2,
	REG_OUTPUT_1 = 3,
	REG_POLARITY_INV_0 = 4,
	REG_POLARITY_INV_1 = 5,
	REG_CONFIGURATION_0 = 6,
	REG_CONFIGURATION_1 = 7
} PCA9555_REGISTERS;

//----------- Master Transmitter/Receiver -------------------
#define TW_START 0x08
#define TW_REP_START 0x10
//---------------- Master Transmitter ----------------------
#define TW_MT_SLA_ACK 0x18
#define TW_MT_SLA_NACK 0x20
#define TW_MT_DATA_ACK 0x28
//---------------- Master Receiver ----------------
#define TW_MR_SLA_ACK 0x40
#define TW_MR_SLA_NACK 0x48
#define TW_MR_DATA_NACK 0x58
#define TW_STATUS_MASK 0b11111000
#define TW_STATUS (TWSR0 & TW_STATUS_MASK)


// global pressed key
volatile uint16_t pressed_key = 0;

// read first time pressed key
uint8_t flag = 0;

//initialize TWI clock
void twi_init(void)
{
	TWSR0 = 0; // PRESCALER_VALUE=1
	TWBR0 = TWBR0_VALUE; // SCL_CLOCK 100KHz
}

// Read one byte from the twi device (request more data from device)
unsigned char twi_readAck(void)
{
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR0 & (1<<TWINT)));
	return TWDR0;
}

//Read one byte from the twi device, read is followed by a stop condition
unsigned char twi_readNak(void)
{
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR0 & (1<<TWINT)));
	return TWDR0;
}

// Issues a start condition and sends address and transfer direction.
// return 0 = device accessible, 1= failed to access device
unsigned char twi_start(unsigned char address)
{
	uint8_t twi_status;
	// send START condition
	TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));
	// check value of TWI Status Register.
	twi_status = TW_STATUS & 0xF8;
	if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) return 1;
	// send device address
	TWDR0 = address;
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wail until transmission completed and ACK/NACK has been received
	while(!(TWCR0 & (1<<TWINT)));
	// check value of TWI Status Register.
	twi_status = TW_STATUS & 0xF8;
	if ( (twi_status != TW_MT_SLA_ACK) && (twi_status != TW_MR_SLA_ACK) )
	{
		return 1;
	}
	return 0;
}

// Send start condition, address, transfer direction.
// Use ack polling to wait until device is ready
void twi_start_wait(unsigned char address)
{
	uint8_t twi_status;
	while ( 1 )
	{
		// send START condition
		TWCR0 = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);
		// wait until transmission completed
		while(!(TWCR0 & (1<<TWINT)));
		// check value of TWI Status Register.
		twi_status = TW_STATUS & 0xF8;
		if ( (twi_status != TW_START) && (twi_status != TW_REP_START)) continue;
		// send device address
		TWDR0 = address;
		TWCR0 = (1<<TWINT) | (1<<TWEN);
		// wail until transmission completed
		while(!(TWCR0 & (1<<TWINT)));
		// check value of TWI Status Register.
		twi_status = TW_STATUS & 0xF8;
		if ( (twi_status == TW_MT_SLA_NACK )||(twi_status ==TW_MR_DATA_NACK) )
		{
			/* device busy, send stop condition to terminate write operation */
			TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
			// wait until stop condition is executed and bus released
			while(TWCR0 & (1<<TWSTO));
			continue;
		}
		break;
	}
}

// Send one byte to twi device, Return 0 if write successful or 1 if write failed
unsigned char twi_write( unsigned char data )
{
	// send data to the previously addressed device
	TWDR0 = data;
	TWCR0 = (1<<TWINT) | (1<<TWEN);
	// wait until transmission completed
	while(!(TWCR0 & (1<<TWINT)));
	if( (TW_STATUS & 0xF8) != TW_MT_DATA_ACK) return 1;
	return 0;
}

// Send repeated start condition, address, transfer direction
//Return: 0 device accessible
// 1 failed to access device
unsigned char twi_rep_start(unsigned char address)
{
	return twi_start( address );
}
// Terminates the data transfer and releases the twi bus
void twi_stop(void)
{
	// send stop condition
	TWCR0 = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	// wait until stop condition is executed and bus released
	while(TWCR0 & (1<<TWSTO));
}
void PCA9555_0_write(PCA9555_REGISTERS reg, uint8_t value)
{
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_write(value);
	twi_stop();
}

uint8_t PCA9555_0_read(PCA9555_REGISTERS reg)
{
	uint8_t ret_val;
	twi_start_wait(PCA9555_0_ADDRESS + TWI_WRITE);
	twi_write(reg);
	twi_rep_start(PCA9555_0_ADDRESS + TWI_READ);
	ret_val = twi_readNak();
	twi_stop();
	return ret_val;
}



void lcd_print(const char* str) {
	while (*str) {
		lcd_data(*str++);
	}
}

void write_2_nibbles(uint8_t data) {
	// uint8_t port_state = LCD_DATA_PIN & 0x0F;  // Preserve lower 4 bits
	
	// // Send high nibble
	// LCD_DATA_PORT = (data & 0xF0) | port_state;
	// LCD_DATA_PORT |= (1 << LCD_E);   // Enable pulse
	// _delay_us(1);
	// LCD_DATA_PORT &= ~(1 << LCD_E);

	// // Send low nibble
	// LCD_DATA_PORT = ((data << 4) & 0xF0) | port_state;
	// LCD_DATA_PORT |= (1 << LCD_E);   // Enable pulse
	// _delay_us(1);
	// LCD_DATA_PORT &= ~(1 << LCD_E);
	
	uint8_t port_state = PCA9555_0_read(REG_OUTPUT_0) & 0x0F;
	// send high nibble
	LCD_DATA_PORT = (data & 0xF0) | port_state;
	PCA9555_0_write(REG_OUTPUT_0,LCD_DATA_PORT);
	LCD_DATA_PORT |= (1 << LCD_E);   // Enable pulse
	PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_PORT);
	_delay_us(1);
	LCD_DATA_PORT &= ~(1 << LCD_E);
	PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_PORT);

	// send low nibble
	LCD_DATA_PORT = ((data << 4) & 0xF0) | port_state;
	PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_PORT);
	LCD_DATA_PORT |= (1 << LCD_E);   // Enable pulse
	PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_PORT);
	_delay_us(1);
	LCD_DATA_PORT &= ~(1 << LCD_E);
	PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_PORT);
}

void lcd_command(uint8_t cmd) {
	LCD_DATA_PORT &= ~(1 << LCD_RS);  // RS = 0 for command
	PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_PORT);
	write_2_nibbles(cmd);
	_delay_us(250);
}

void lcd_data(uint8_t data) {
	LCD_DATA_PORT |= (1 << LCD_RS);   // RS = 1 for data
	PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_PORT);
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
	PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_DDR);

	_delay_ms(200);
	
	// Initialize LCD in 4-bit mode
	// Send 0x30 three times
	for (uint8_t i = 0; i < 3; i++) {
		LCD_DATA_PORT = 0x30;
		LCD_DATA_PORT |= (1 << LCD_E);
		PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_PORT);
		_delay_us(1);
		LCD_DATA_PORT &= ~(1 << LCD_E);
		PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_PORT);
		_delay_us(250);
	}
	
	// Switch to 4-bit mode
	LCD_DATA_PORT = 0x20;
	LCD_DATA_PORT |= (1 << LCD_E);
	PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_PORT);
	_delay_us(1);
	LCD_DATA_PORT &= ~(1 << LCD_E);
	PCA9555_0_write(REG_OUTPUT_0, LCD_DATA_PORT);
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

uint16_t scan_row(uint16_t row){
	uint16_t temp = 0;
	uint16_t input = 0;

	// read output pins and change only
	// the row i want to 0 (com logic)
	// temp = PCA9555_0_read(REG_OUTPUT_1);
	PCA9555_0_write(REG_OUTPUT_1, (0xFF & ~(1 << row)));
	temp = PCA9555_0_read(REG_INPUT_1);
	//PORTD = temp;

	for(uint8_t i = 0; i < 4; i++){
		// columns are in temp
		// 0bc3 c2 c1 c0 rows...
		// gia afto thelw shift left
		// gia na parw tis steiles
		if(!(temp & (1 << (i + 4))))
		input |= 1 << (row * 4 + i);
	}
	// restore default value of rows
	PCA9555_0_write(REG_OUTPUT_1, 0xFF);
	return input;
}

uint16_t scan_keypad(void){
	uint16_t answer = 0;

	for(uint8_t i = 0; i < 4; i++){
		answer |= scan_row(i);
	}
	
	return answer;
}

uint16_t scan_keypad_rising_edge(void){
	uint16_t pressed_keys_tempo = 0;

	if (flag == 0){
		pressed_key = scan_keypad();
		flag = 1;
	}
	_delay_ms(20);
	pressed_keys_tempo = scan_keypad();
	_delay_ms(20);
	pressed_keys_tempo &= scan_keypad();
	
	// keep only the new (debounce - free) value
	uint16_t return_value = pressed_keys_tempo & ~pressed_key;
	//pressed_key = pressed_keys_tempo;
	return return_value;

}

int pressed_keys(void){
	uint16_t input = 0;
	int counter = 0;

	input = scan_keypad_rising_edge();

	while((!(input & 1)) && counter < 16){
		input = input >> 1;
		counter ++;
	}
	return counter;
}

uint8_t keypad_to_ascii(void){
	int counter = 0;
	// keyboard
	char keypad[17] = {
		'*','0','#','D',
		'7','8','9','C',
		'4','5','6','B',
		'1','2','3','A',' '
	};
	
	//_delay_ms(1000);
	counter = pressed_keys();
	
	//char buffer[4];  // Buffer to hold string (max 3 digits + null terminator)
	//sprintf(buffer, "%u", counter);  // %u for unsigned integer
	//lcd_print(buffer);
	
	//lcd_data(keypad[1]);
	//lcd_data(keypad[counter]);
	
	return keypad[counter];
}


int main(void){
	twi_init();

	// set port0 as output
	// and port1[3:0] as output and port1[7:4] as input
	PCA9555_0_write(REG_CONFIGURATION_0, 0xFF);
	PCA9555_0_write(REG_CONFIGURATION_1, 0b11110000);
	
	// initialize PORTB as output
	// and turn off all leds
	DDRB = 0xFF;
	PORTB = 0x00;
	
	DDRD = 0xFF;
	PORTD = 0x00;

	uint8_t output = keypad_to_ascii();
	uint8_t temp_output = 0;
	while(1){
		PORTB = 0x00;
		if (output == '4') PORTB = 0b00000010;
		if (output == '2') PORTB = 0b00000100;
		if (output == '3') PORTB = 0b00001000;
		if (output == 'B') PORTB = 0b00010000;
		
		// stay here until input changes		
		temp_output = keypad_to_ascii();
		while(temp_output == output)
			temp_output = keypad_to_ascii();
		output = temp_output;
	}
}