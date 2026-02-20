.include "m328pbdef.inc"


.equ FOSC_MHZ = 16				;operating frequency in MHz
.equ DEL_mS = 500				;delay in ms (valid number from 1 to 4095)
.equ DEL_NU = FOSC_MHZ * DEL_mS	;delay_mS routine: (1000 * DEL_NU + 6) cycles

.equ PD1 = 1
.equ PD3 = 3

.def counter = r18

.org 0x00
rjmp reset

.org 0x04
rjmp isr1

reset:
;Init Stack pointer
	ldi r24, LOW(RAMEND)	;initializes stack pointer
	out SPL, r24			;to the highest valid address
	ldi r24, HIGH(RAMEND)	;of SRAM so we dont 
	out SPH, r24			;overwrite random memory

;Init PORTB and PORTC as output
	ser r26
	out DDRB, r26
	ser r26
	out DDRC, r26

;Init PORTD as input
	cbi DDRD, PD1
	cbi DDRD, PD3

;set interruption
	ldi r24, (1 << ISC11) | (1 << ISC10)
	sts EICRA, r24

;enable interruption
	ldi r24, (1 << INT1)
	out EIMSK, r24

	sei

;set counter to zero
	ldi counter, 0x00

	rjmp loop1

isr1:
	push r16
	push r17
	push r24
	push r25
	push r26
	push r27
	in r27, SREG
	push r27

	loop3_debouncing:
		ldi r24, LOW(0010)
		ldi r25, HIGH(0010)

		ldi r16, (1 << INTF1)
		out EIFR, r16
	
		;delay for 5ms
		wait_x_msec:
			msec_loop:
				ldi r26, LOW(3998)    ; inner loop counter for 1 msec
				ldi r27, HIGH(3998)

			inner_loop:
				sbiw r26, 1           ; 2 cycles
				brne inner_loop       ; 2 cycles (1 if no branch)

			sbiw r24, 1
			brne wait_x_msec

		in r16, EIFR
		sbrc r16, INTF1
		rjmp loop3_debouncing

	main_loop:
		in r17, PIND
		# may need to insert 
		com r17
		# cpi r17, 0b00001010
		sbrc r17, 1
		rjmp next

		inc counter
		cpi counter, 0b00100000
		brne output

		ldi counter,0x00
		output:
			mov r16, counter
			lsl r16
			out PORTC,r16

		# rcall delay_mS

	next:
		pop r27
		out SREG, r27 
		pop r27
		pop r26
		pop r25
		pop r24
		pop r17
		pop r16

	reti
		
loop1:
	# rjmp loop1
	clr r26
loop2:
	out PORTB, r26

	ldi r24, low(DEL_NU)
	ldi r25, high(DEL_NU)
	rcall delay_mS

	inc r26

	cpi r26,16
	breq loop1
	rjmp loop2

delay_mS:
	ldi r23, 249

	loop_inn:
		dec r23
		nop
		brne loop_inn

		sbiw r24,1
		brne delay_mS

		ret