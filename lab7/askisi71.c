#define F_CPU 16000000UL  // 16MHz
#include <avr/io.h>
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


uint8_t one_wire_reset(void) {
	uint8_t presence;
	
	// Set PD4 as output and pull low
	DDRD |= (1 << PD4);
	PORTD &= ~(1 << PD4);
	
	// 480 usec reset pulse
	_delay_us(480);
	
	// Set PD4 as input and disable pull-up
	DDRD &= ~(1 << PD4);
	PORTD &= ~(1 << PD4);
	
	// Wait 100 usec for connected devices to transmit presence pulse
	_delay_us(100);
	
	// Read PORTD to check presence
	presence = PIND;
	
	// Wait for 380 usec
	_delay_us(380);
	
	// If device detected (PD4 = 0), return 1, else return 0
	if (presence & (1 << PD4)) {
		return 0;  // No device
		} else {
		return 1;  // Device present
	}
}

uint8_t one_wire_receive_bit(void) {
	uint8_t bit;
	
	// Set PD4 as output and pull low
	DDRD |= (1 << PD4);
	PORTD &= ~(1 << PD4);
	
	// Time slot 2 usec
	_delay_us(2);
	
	// Set PD4 as input and disable pull-up
	DDRD &= ~(1 << PD4);
	PORTD &= ~(1 << PD4);
	
	// Wait 10 usec
	_delay_us(10);
	
	// Read PD4
	if (PIND & (1 << PD4)) {
		bit = 1;
		} else {
		bit = 0;
	}
	
	// Delay 49 usec to meet the standards
	_delay_us(49);
	
	return bit;
}

void one_wire_transmit_bit(uint8_t bit) {
	// Set PD4 as output and pull low
	DDRD |= (1 << PD4);
	PORTD &= ~(1 << PD4);
	
	// Time slot 2 usec
	_delay_us(2);
	
	// Set PD4 based on bit value
	if (bit & 0x01) {
		PORTD |= (1 << PD4);   // Set PD4 high if bit is 1
		} else {
		PORTD &= ~(1 << PD4);  // Keep PD4 low if bit is 0
	}
	
	// Wait 58 usec for device to sample the line
	_delay_us(58);
	
	// Set PD4 as input and disable pull-up (release line)
	DDRD &= ~(1 << PD4);
	PORTD &= ~(1 << PD4);
	
	// Recovery time 1 usec
	_delay_us(1);
}

uint8_t one_wire_receive_byte(void) {
	uint8_t byte_value = 0;
	uint8_t bit;
	
	for (uint8_t i = 0; i < 8; i++) {
		bit = one_wire_receive_bit();
		
		byte_value >>= 1;  // Shift right
		
		if (bit & 0x01) {
			byte_value |= 0x80;  // Set MSB if bit is 1
		}
	}
	
	return byte_value;
}

void one_wire_transmit_byte(uint8_t byte_value) {
	uint8_t temp = byte_value;
	
	for (uint8_t i = 0; i < 8; i++) {
		one_wire_transmit_bit(temp & 0x01);  // Transmit LSB
		temp >>= 1;  // Shift right for next bit
	}
}

uint8_t read_temperature(uint8_t *temp_lsb, uint8_t *temp_msb){
	if (!one_wire_reset())
	return 0x8000;
	
	one_wire_transmit_byte(0xCC);
	one_wire_transmit_byte(0x44);

	// wait until calculation is over
	while(!one_wire_receive_bit());

	if (!one_wire_reset())
		return 0x8000;
	one_wire_transmit_byte(0xCC);
	// to read temperature
	one_wire_transmit_byte(0xBE);

	//Read the temperature bytes
	*temp_lsb = one_wire_receive_byte();
	*temp_msb = one_wire_receive_byte();

	return 1;
}

int main(void){
	DDRD = 0xff;
	uint8_t lsb,msb;
	uint8_t temp_decimal;
	uint16_t temp_integer,temperature;
	char buffer[32];

	twi_init();
	lcd_init();

	while(1){
		if(read_temperature(&lsb,&msb)){
			// sensor DS18B20
			
			//temperature = ((uint16_t)msb << 8) | lsb;
			temperature = msb;
			temperature <<= 8;
			temperature |= lsb;
			// *************************************************
			// check if it works this this value of temperature
			// if not then try ~(1 << 15) in if statement
			// *************************************************
			//temperature = 0x0192;

			temp_integer = (temperature >> 4) & 0b01111111;
			temp_decimal = ((temperature & 0x0F) * 625) / 100;  // * 0.0625 * 100
			
			if (temperature & ~(1<<15)) {
				sprintf(buffer, "%d.%01d%cC", temp_integer, temp_decimal / 10,0xDF);
				} else {
				// Για αρνητικές θερμοκρασίες
				sprintf(buffer, "-%d.%01d%cC", abs(temp_integer), temp_decimal / 10,0xDF);
			}
			
			//sprintf(buffer, "%u, %u, %u", lsb, msb, temperature);
			lcd_clear();
			lcd_print(buffer);

			}else{
			lcd_clear();
			lcd_print("No Device");
		}
		_delay_ms(1000);
	}

}