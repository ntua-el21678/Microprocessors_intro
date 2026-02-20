.include "m328pbdef.inc"
.def temp = r16
.def dc_value = r17
.def table_index = r18

.org 0x00
    rjmp reset

reset:
	;Init Stack pointer
	ldi r24, LOW(RAMEND)	;initializes stack pointer
	out SPL, r24			;to the highest valid address
	ldi r24, HIGH(RAMEND)	;of SRAM so we dont 
	out SPH, r24			;overwrite random memory

    ;set output/input
    ; PB1 = output (PWM), PB4 & PB5 = input (buttons)
	;ldi temp, (1 << 1)           ; only PB1 = 1
	;sts DDRB, temp

    ldi temp, 0b11001111
    out DDRB, temp

	ldi temp, 0x00
	out PORTB,temp


    ;clk/1024 => 62500Hz and fast non inverting pwm - 8 bit
    ldi temp,(1 << WGM10) | (1 << COM1A1)    
    sts TCCR1A, temp
    ldi temp, (1 << CS12) | (1 << CS10) | (1 << WGM12)  
    sts TCCR1B, temp

    ;set duty cycle to 50%
    ldi table_index, 9
	rcall update_table
	sts OCR1AL, dc_value

    rjmp main

main:
    ;read input PB4 & PB5	
	in temp, PINB
	com temp
	sbrc temp,4
    rjmp increase_duty_cycle

    in temp, PINB
    com temp
	sbrc temp,5
    rjmp decrease_duty_cyle

    rjmp main
    
increase_duty_cycle:
	in temp, PINB
	com temp
	sbrc temp,4
    rjmp increase_duty_cycle

    cpi table_index, 16
    breq continue   ; stay at max
    inc table_index
    rjmp continue

decrease_duty_cyle:
	in temp, PINB
    com temp
	sbrc temp,5
    rjmp decrease_duty_cyle

    cpi table_index, 0
    breq continue   ; stay at min
    dec table_index
    rjmp continue
    
continue:
    ldi ZH, HIGH(levels*2)
    ldi ZL, LOW(levels*2)
    add ZL, table_index

    lpm dc_value, Z
    sts OCR1AL, dc_value

    rjmp main
	
update_table:
    ldi ZH, HIGH(levels*2)
    ldi ZL, LOW(levels*2)

    ;ldi temp, 0
    add ZL,table_index
    ;adc ZH, temp
	lpm dc_value,Z
    ret

levels:
	.db 5,20,36,51,66,82,97,112,128,143,158,173,189,204,219,235,250,0