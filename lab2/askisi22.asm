.include "m328pbdef.inc"

.def counter = r20
.def lowtimer = r24
.def hightimer = r25
.equ PD2 = 2

.org 0x00
rjmp reset

.org 0x02
rjmp isr0

reset:
	; Initialize stack
	ldi r16, LOW(RAMEND)
	out SPL, r16
	ldi r16, HIGH(RAMEND)
	out SPH, r16

	;Enable interrupts
	ldi r26, (1 << ISC01) | (1 << ISC00)
	sts EICRA, r26

	ldi r26, (1 << INT0)
	out EIMSK, r26

	sei

	;Set PORTC as output and PORTB as input
	ldi r16, 0xFF
	out DDRC, r16
	ldi r16, 0x00
	out DDRB, r16

	cbi DDRD, PD2      ; PD2 = INT0

	;set counter to 0
	ldi counter, 0

	;set timer to 1 second
	ldi lowtimer, LOW(1000)
	ldi hightimer, HIGH(1000)
	
	rjmp main_loop

main_loop:
	mov r16, counter
	lsl r16
	out PORTC, r16
	rcall wait_1_sec

	inc counter
	cpi counter, 0b00010000
	brne next

	;reset counter
	ldi counter, 0

	next:
		rjmp main_loop

isr0:
	push r16
	push r17
	push r18
	in r18, SREG
	push r18

	in r16, PINB
	com r16
	andi r16,0b00001111

	;count pressed buttons
	ldi r17,4
	ldi r18,0		;counter
	count_buttons:
		sbrs r16,0
		rjmp next1

		lsl r18
		ori r18, 0b00000001

		# enallaktikos tropos
		# thetoume to carry iso me 1 
		# kai meta kanoume rotate through carry
		
		# sec
		# rol r18

	next1:
		lsr r16
		dec r17
		brne count_buttons

	;print to portc
	;lsl r18
	out PORTC, r18
	rcall wait_1_sec
	rcall wait_1_sec
	
	pop r18
	out SREG, r18
	pop r18
	pop r17
	pop r16

	reti


wait_1_sec:
	msec_loop:
		ldi r26, LOW(3998)    ;3998 inner loop counter for 1 msec
		ldi r27, HIGH(3998)

		inner_loop:
			sbiw r26, 1           ; 2 cycles
			brne inner_loop       ; 2 cycles (1 if no branch)

		sbiw lowtimer, 1
		brne msec_loop

	;set timer again to 1 second
	ldi lowtimer, LOW(1000)
	ldi hightimer, HIGH(1000)

	ret
