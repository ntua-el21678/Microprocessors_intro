.include "m328pbdef.inc" 

.org 0x00
rjmp reset                ; reset vector

.def lowbit = r24
.def highbit = r25


;1 cycles -> 62.5 ns
;loop has 4 cycles
;loop takes 4 cycles x 62.5 ns = 250 ns per loop
;max delay (lowbit = highbit = 0xFF) => 65,535 iterations x 250 ns = 16 ms max delay
;to have 1 ms delay => 16 000 cycles 

reset:
	; --- 1. ΑΡΧΙΚΟΠΟΙΗΣΗ STACK POINTER (Απαραίτητο!) ---
    ldi r16, LOW(RAMEND)
    out SPL, r16
    ldi r16, HIGH(RAMEND)
    out SPH, r16

	ldi lowbit,low(0005)
	ldi highbit,high(0005)

	rcall wait_x_msec

	rjmp end

wait_x_msec:
	

	msec_loop:
    ldi r26,LOW(3998)    ; inner loop counter for 1 msec
    ldi r27,HIGH(3998)

	inner_loop:
    sbiw r26, 1           ; 2 cycles
    brne inner_loop       ; 2 cycles (1 if no branch)

	sbiw lowbit, 1
	brne wait_x_msec

	ret

end:
	rjmp end

