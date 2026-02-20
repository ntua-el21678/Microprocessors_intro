.include "m328pbdef.inc"

.def flag = r20
.def output = r23
.def lowtimer = r24
.def hightimer = r25

.org 0x00
	rjmp reset
.org 0x04 
	rjmp ISR1
    
reset:
    ldi r16,LOW(RAMEND)
    out SPL,r16
    ldi r16,HIGH(RAMEND)
    out SPH,r16
    
    ;interrupt enable
    ldi r16, (1 << ISC11) | (0 << ISC10)
    sts EICRA,r16
	ldi r16, (1 << INT1)
    out EIMSK,r16
    
	clr r16
    out DDRD,r16
    ser r16
    out DDRB,r16

	ldi flag, 0x00
	rjmp main
   
main:
	cpi flag, 1
	breq pressed
	rjmp main


pressed:
	cbr flag, (1 << 0) 

    cpi output, (1 << 3)
    breq exception
	cpi output, 0xFF
	breq exception

    ldi lowtimer,LOW(4000)
    ldi hightimer,HIGH(4000)
    rjmp standard
    
	exception:    
		ldi lowtimer,LOW(1000)
		ldi hightimer,HIGH(1000)
    
		ldi output,0xFF
		out PORTB,output
		
		rcall wait_x_msec
		cpi flag, (1 << 0)
		breq pessed

		ldi lowtimer,LOW(3000)
		ldi hightimer,HIGH(3000)
    
		rjmp standard

    standard:
		ldi output, (1 << 3)
		out PORTB,output
		rcall wait_x_msec

		clr output
		out PORTB,output
rjmp main
    
ISR1:
	push r24            
    push r25            
    push r26            
    push r27            
    push r16
    in r16, SREG
    push r16

	rcall debouncing
    sbr flag, (1 << 0)

    pop r16
    out SREG, r16
    pop r16
	pop r27
    pop r26
    pop r25
    pop r24

    reti

debouncing:
	ldi lowtimer, LOW(0015)
	ldi hightimer, HIGH(0015)

	ldi r16, (1 << INTF1)
	out EIFR, r16
	
	;delay for 5ms
	rcall wait_x_msec

	in r16, EIFR
	sbrc r16,0
	brne debouncing
ret

wait_x_msec:
    ; Total cycles needed for 1 msec at 16MHz = 16000 cycles
    ; Each loop iteration = 4 cycles (sbiw + brne)
	msec_loop:
		ldi r26, LOW(3998)    ; inner loop counter for 1 msec
		ldi r27, HIGH(3998)

	inner_loop:
		cpi flag, (1 << 0)
		breq exit

		sbiw r26, 1           ; 2 cycles
		brne inner_loop       ; 2 cycles (1 if no branch)

    sbiw lowtimer, 1           ; 2 cycles, decrement msec counter
    brne msec_loop        ; 2 cycles, loop for next msec
	exit:
ret
