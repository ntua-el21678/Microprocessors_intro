.include "m328pbdef.inc" 

.org 0x00
.def lowbit = r24
.def highbit = r25
.def output = r20

rjmp reset                ; reset vector

reset:
	ldi r16,0xFF
	out DDRD,r16

	ldi r17,0x00
	bst r17,0

	;0 bit -> LSB to MSB (starting state)
	;1 bit -> MSB to LSB

	ldi output,0b00000001
	out PORTD,output
	
	rcall wait_3_msec
	rjmp loop


loop:
	bld r17,0
	;check if T=1
	sbrs r17,0 ;skip next instruction if T=1
	rjmp go_left

	go_right:
		lsr output
		rjmp continue

	go_left:
		lsl output
	
	continue:
		out PORTD,output
		
		cpi output,0b00000001
		breq is_at_lsb
		cpi output,0b10000000
		breq is_at_msb

		rcall wait_2_msec
		rjmp loop

		is_at_lsb:
			ldi r17,0b00000000
			rjmp next
			
		is_at_msb:
			ldi r17,0b00000001
		
		next:
		bst r17,0
		rcall wait_3_msec
		rjmp loop

wait_2_msec:
	
	;ldi r26,122
	
	;inner loop delay = 16ms
	;122 times x 16 ms = 2 seconds
	;inner_loop_1:
		ldi lowbit,low(2000)
		ldi highbit,high(2000)
		rcall wait_x_msec

	;dec r26
	;brne inner_loop_1

	ret

wait_3_msec:
	;ldi r26,183

	;183 times x 16 ms = 3 seconds
	;inner_loop:
		ldi lowbit,low(3000)
		ldi highbit,high(3000)
		rcall wait_x_msec

	;dec r26
	;brne inner_loop 

	ret

;4 cycles
;at 16Mhz -> 1 cycle=62.5ns
;whole loop => 4 cycles x 62.5 ns = 250ns each loop
;16 000 cycles for 1ms delay
wait_x_msec:
	
	msec_loop:
    ldi r26, LOW(3998)    ; inner loop counter for 1 msec
    ldi r27, HIGH(3998)

	inner_loop:
    sbiw r26, 1           ; 2 cycles
    brne inner_loop       ; 2 cycles (1 if no branch)

	sbiw lowbit, 1
	brne wait_x_msec

	ret

	sbiw lowbit, 1
	brne wait_x_msec

	ret

end:
	rjmp end