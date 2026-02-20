#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdio.h>

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

// global variables
volatile uint8_t flag = 0;
volatile uint16_t pressed_key = 0;

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


// -------------------------- //
// -------- Start PCA ------- //
// -------------------------- //

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
// -------------------------- //
// -------- End PCA --------- //
// -------------------------- //


// -------------------------- //
// -------- Start LCD ------- //
// -------------------------- //

// prints strings
// usage: lcd_print("Hello");
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

// prints chars
// usage: lcd_data('A');
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
// -------------------------- //
// -------- End LCD --------- //
// -------------------------- //


// -------------------------- //
// -------- Start UART ------ //
// -------------------------- //

// Υπολογισμός UBRR για 9600 baud rate στα 16MHz
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1) // = 103

// --- 1. Initialize UART ---
void usart_init(unsigned int ubrr){
	UCSR0A=0;
	UCSR0B=(1<<RXEN0)|(1<<TXEN0);
	UBRR0H=(unsigned char)(ubrr>>8);
	UBRR0L=(unsigned char)ubrr;
	UCSR0C=(3 << UCSZ00);
	
	return;
}

// --- 2. Send 1 byte ---
void usart_transmit(uint8_t data){
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0=data;
}

// --- 3. Recieve 1 byte ---
uint8_t usart_receive(){
	while(!(UCSR0A&(1<<RXC0)));
	return UDR0;
}

// --- Send string to ESP ---
void usart_print(const char *str) {
	while (*str) {
		usart_transmit(*str++);
	}
}

// --- Read String until '\n' ---
void usart_receive_string(char *buffer, int max_len) {
	char received_char;
	int i = 0;
	
	for (i = 0;i < max_len - 1; i++) {
		received_char = usart_receive();
		// Τα μηνύματα του ESP τελειώνουν με '\n'
		if (received_char == '\n') {
			break;
		}
		buffer[i] = received_char;
	}
	buffer[i] = '\0';
}
// -------------------------- //
// -------- End UART -------- //
// -------------------------- //



// ----------------------------------- //
// -------- Start Temperature -------- //
// ----------------------------------- //
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
// --------------------------------- //
// -------- End Temperature -------- //
// --------------------------------- //



// --------------------------------- //
// ---------- Start Keypad --------- //
// --------------------------------- //
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
	
	return keypad[counter];
}

// --------------------------------- //
// ----------- End Keypad ---------- //
// --------------------------------- //

void calc_adc(float *voltage_adc, uint16_t *blood_pressure) {
	// Start ADC conversion
	ADCSRA |= (1 << ADSC);
	// wait for adc to finish
	while (ADCSRA & (1 << ADSC));
	
	// Read ADC result (10-bit)
	uint16_t adc_value = ADC;  // ADCL and ADCH combined
	
	// Calculate voltage: (adc * 5.0) / 1024
	*voltage_adc = ((float)adc_value * 5.0) / 1024.0;
	
	// Calculate result: 4 * (voltage - 0.1)
	float temp_result = 4 * (*voltage_adc - 0.1);
	
	// Clamp to positive values
	if (temp_result < 0)
	*blood_pressure = 0;
	else
	*blood_pressure = (uint16_t)temp_result;
}

void initialize (){
	// uart
	usart_init(UBRR_VALUE);
	// twi
	twi_init();
	// lcd screen
	lcd_init();
	
	// ADC
	// Set ADC reference to AVCC (5V) and select ADC0 (PC0)
	ADMUX = (1 << REFS0) | (0 << MUX0) | (0 << MUX1) | (0 << MUX2) | (0 << MUX3);
	// Enable ADC, Enable ADC interrupt, Set prescaler to 128 (125kHz @ 16MHz)
	ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	// set port0 as output
	// and port1[3:0] as output and port1[7:4] as input
	PCA9555_0_write(REG_CONFIGURATION_0, 0x00);
	PCA9555_0_write(REG_CONFIGURATION_1, 0b11110000);

	_delay_ms(1000); // Μικρή καθυστέρηση εκκίνησης
}

int main(void) {
	char response_buffer[64]; // Buffer to save data
	char *status = "OK";
	char payload_buffer[256]; // Μεγάλος buffer για να χωρέσει όλο το JSON
	char print_data[32];

	uint8_t lsb = 0,msb = 0;
	uint8_t temp_decimal = 0;
	uint16_t temp_integer = 0,temperature = 0;
	float voltage_adc = 0;
	uint16_t blood_pressure = 0;
	uint8_t keypad_output = 0;
	uint8_t keypad_temp_output = 0;
	uint8_t first_digit = 0;
	uint8_t second_digit = 0;
	uint8_t i = 0;

	// ------- initialize -------
	initialize();

	lcd_clear();
	lcd_print("Waiting ...");
	// --- Step 1: Connect ---
	usart_print("ESP:connect\n");

	// clear buffer before a response is saved
	memset(response_buffer, 0, sizeof(response_buffer));
	usart_receive_string(response_buffer, 64);
	
	lcd_clear();
	lcd_print("Received answer");
	_delay_ms(500);
	lcd_clear();
	
	//lcd_print("Initialize ESP: ");
	if (strstr(response_buffer, "Success")) {
		lcd_print("1.Success");
		} else {
		lcd_print("1.Fail");
	}
	
	_delay_ms(3000); // Καθυστέρηση για να προλάβετε να δείτε το μήνυμα
	
	lcd_clear();
	lcd_print("ESP:URL");

	// --- Step 2: Send target url ---
	usart_print("ESP:url:\"http://192.168.1.250:5000/data\"\n");

	// clear buffer before a response is saved
	memset(response_buffer, 0, sizeof(response_buffer));
	usart_receive_string(response_buffer, 64);
	
	lcd_clear();
	//lcd_print("Connect to url: ");
	//try lcd_print("response buffer")
	if (strstr(response_buffer, "Success")) {
		lcd_print("2.Success");
		} else {
		lcd_print("2.Fail");
	}
	
	_delay_ms(1000);
	lcd_clear();
	
	DDRD = 0xFF;
	while (1) {
		if(read_temperature(&lsb,&msb)){
			// initialize status
			// calculate temperature
			temperature = msb;
			temperature <<= 8;
			temperature |= lsb;
			
			temp_integer = (temperature >> 4) & 0b01111111;
			temp_decimal = ((temperature & 0x0F) * 625) / 100;  // * 0.0625 * 100
			
			// add ~10 to temp_integer to approximate 36 degrees
			temp_integer +=13;

			// calculate blood pressure through pt0
			calc_adc(&voltage_adc, &blood_pressure);

			// read from keypad
			keypad_temp_output = keypad_to_ascii();
			while((keypad_temp_output == keypad_output && keypad_output != ' ')){
				keypad_temp_output = keypad_to_ascii();
			}
			//lcd_clear();
			keypad_output = keypad_temp_output;
			
			if (keypad_output == '#')
				status = "OK";
			if (keypad_output == '1')
				status = "NURSE CALL";
			if (status == "NURSE CALL")
				status = "NURSE CALL";
			else{
				status = "OK";
				if (blood_pressure > 12 || blood_pressure < 4)
					status = "CHECK PRESSURE";
				if (temp_integer >= 37 || temp_integer < 34)
					status = "CHECK TEMP";
				if ((blood_pressure > 12 || blood_pressure < 4) && (temp_integer >= 37 || temp_integer <= 34))
					status = "CHECK PRESS&TEMP";
			}
		}

		// create payload
		sprintf(payload_buffer,
		"ESP:payload:[{\"name\": \"temperature\",\"value\": \"%d.%d\"},"
		"{\"name\": \"pressure\",\"value\": \"%d.%d\"},"
		"{\"name\": \"team\",\"value\": \"71\"},"      
		"{\"name\": \"status\",\"value\": \"%s\"}]\n",
		temp_integer, temp_decimal,
		blood_pressure, (blood_pressure % 10),
		status
		);

		if (temperature & ~(1<<15)) {
			sprintf(print_data, "%d.%01d%cC %d.%d", temp_integer, temp_decimal / 10, 0xDF, blood_pressure, blood_pressure % 10);
			} else {
			// Για αρνητικές θερμοκρασίες
			sprintf(print_data, "-%d.%01d%cC %d.%d", abs(temp_integer), temp_decimal / 10, 0xDF, blood_pressure, blood_pressure % 10);
		}
		lcd_print(print_data);
		lcd_command(0xC0); // Μετακίνηση κέρσορα στην αρχή της 2ης γραμμής
		lcd_print(status);
		
		usart_print(payload_buffer);
		// Καθαρισμός και αναμονή για το "Success" του ESP
		memset(response_buffer, 0, 64);
		usart_receive_string(response_buffer, 64);
		
		_delay_ms(2000);
		lcd_clear();
		lcd_print("3.");
		lcd_print(response_buffer);
		_delay_ms(2000);
		
		// --- Εντολή Transmit ---
		usart_print("ESP:transmit\n");

		// waiting for server response
		memset(response_buffer, 0, 64);
		usart_receive_string(response_buffer, 64);

		lcd_clear();
		lcd_print("4.");
		lcd_print(response_buffer);
		
		_delay_ms(3000);
		lcd_clear();
	}
}