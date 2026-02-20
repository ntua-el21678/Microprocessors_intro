.include "m328PBdef.inc"

.def timerlow = r24
.def timerhigh = r25
.def temp = r16
.def reg1 = r17
.def reg2 = r18
.def reg3 = r19
.def adcll = r22
.def adchh = r23

.equ PD0=0
.equ PD1=1
.equ PD2=2
.equ PD3=3
.equ PD4=4
.equ PD5=5
.equ PD6=6
.equ PD7=7

.org 0x00
	rjmp reset

.org 0x2A
	rjmp adc_interrupt

reset:
	; init stack pointer
	ldi temp, high(RAMEND)
	out SPH, temp
	ldi temp, low(RAMEND)
	out SPL, temp

	clr r1

	ser r24
	out DDRD, r24

	;init lcd display
	rcall lcd_init

	;delay 1 sec
	ldi timerlow, low(1000)
	ldi timerhigh, high(1000)
	rcall wait_msec

	;init ADC
	ldi temp,0b01000011; Read from A3 (PC3)
	sts ADMUX, temp

	ldi temp,0b10001111
	sts ADCSRA, temp

	; Enable global interrupts
	sei

main:
	;set ADSC flag to start convesion
	lds temp, ADCSRA
	ori temp, (1 << ADSC)
	sts ADCSRA, temp

	ldi timerlow, low(500)
	ldi timerhigh, high(500)
	rcall wait_msec
	
    rjmp main

adc_interrupt:
	; save adc result in
	; registers adcll and adchh
	lds adcll, ADCL
	lds adchh, ADCH

    ; 100*Vin = (ADC*125)/256
    clr r18
    clr r19
	clr r1
	clr r0

    ldi r20, 125

	;delete later
	;ldi adcll, low(423)
	;ldi adchh, high(423)
	;;;;;;;;;;;

    mul adcll, r20          ; Result in r1:r0
    mov r18, r1				; /256. r19:r18 = (ADC low Byte * 125)/256
    mul adchh, r20          ; Result in r1:r0
    add r18, r0
    adc r19, r1             ; add with carry

    ; r19:r18 = 100Vin

    rcall div16by8        ; seperate quotient from remainder

	rcall lcd_clear_display
	
    mov r24, reg2         ; Load quotient
    ori r24, 0x30         ; Convert to ASCII
    rcall lcd_data        ; Print "x"

    ldi r24, '.'
    rcall lcd_data        ; Print "."

    mov r24, reg1          ; Get remainder (cents)
    ldi r22, 10
    rcall div8by8         ; Split into digits

	mov r24, r16
    ori r24, 0x30
    rcall lcd_data        ; Print first "y"

    ori reg1, 0x30
    mov r24, reg1
    rcall lcd_data        ; Print second "y"

    ldi r24, 'V'
    rcall lcd_data        ; Print second "y"

reti

;;;;;;;;;;;;;;;;;;;;;;;;;;Div 8 bits by 8bits;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
div8by8:
	clr r16
	mov reg1,r24

	div8by8_loop:
		cpi reg1, 10
		brlo div8by8_done

		subi reg1,10
		inc r16
		rjmp div8by8_loop

	div8by8_done:
ret

;;;;;;;;;;;;;;;;;;;;;;;;;;Div 16 bits by 8bits;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
div16by8:
    clr r16              ; quotient counter = 0

	div_loop:
		cpi r18, 100
		brlo dec_highbit
		inc r16
		subi r18, 100
		rjmp div_loop

		dec_highbit:
		cpi r19, 0
		breq div_done
		
		dec r19
		inc r16              ; increment quotient
		inc r16
		ldi r17, 56
		add r18,r17
		rjmp div_loop        ; repeat
    
	div_done:
		mov reg1, r18         ; remainder goes to reg1
		mov reg2, r16         ; quotient goes to reg2
ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;LCD Functions;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
write_2_nibbles:
	push r24

	in r25 ,PIND		; read PIND of lcd display
	andi r25 ,0x0f		
	andi r24 ,0xf0		; r24[3:0] Holds previus PORTD[3:0]
	add r24 ,r25		; r24[7:4] <-- LCD_Data_High_Byte
	out PORTD ,r24 

	sbi PORTD ,PD3 ; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
	
	pop r24				; Recover r24(LCD_Data)
	swap r24 
	andi r24 ,0xf0		; r24[3:0] Holds previus PORTD[3:0]
	add r24 ,r25		; r24[7:4] <-- LCD_Data_Low_Byte
	out PORTD ,r24
	sbi PORTD ,PD3		; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
ret

lcd_init:
	ldi timerlow, low(200)
	ldi timerhigh, high(200)
	rcall wait_msec

	ldi temp, 0x03
	loop_1:
		ldi r24 ,0x30 ; command to switch to 8 bit mode
		out PORTD ,r24 ;
		sbi PORTD ,PD3 ; Enable Pulse
		nop
		nop
		cbi PORTD ,PD3
		ldi timerlow, 250
		ldi timerhigh, 0 ; wait 250 usec
		rcall wait_usec 

		dec temp
		cpi temp,0
		brne loop_1

	ldi r24 ,0x20 ; command to switch to 4 bit mode
	out PORTD ,r24
	sbi PORTD ,PD3 ; Enable Pulse
	nop
	nop
	cbi PORTD ,PD3
	ldi timerlow, 250
	ldi timerhigh, 0 ; wait 250 usec
	rcall wait_usec 

	ldi r24 ,0x28 ; 5x8 dots, 2 lines
	rcall lcd_command
	ldi r24 ,0x0c ; dislay on, cursor off
	rcall lcd_command
	rcall lcd_clear_display
	ldi r24 ,0x06 ; Increase address, no display shift
	rcall lcd_command ;
ret
	
lcd_clear_display:
	ldi r24 ,0x01 ; clear display command
	rcall lcd_command
	ldi timerlow ,low(5) ;
	ldi timerhigh ,high(5) ; Wait 5 mSec
	rcall wait_msec ;
ret

lcd_command:
	cbi PORTD ,PD2 ; LCD_RS=0(PD2=0), Instruction
	rcall write_2_nibbles ; send Instruction
	ldi timerlow, 250
	ldi timerhigh, 0; wait 250 usec
	rcall wait_usec
ret

lcd_data:
	sbi PORTD ,PD2 ; LCD_RS=1(PD2=1), Data
	rcall write_2_nibbles ; send Data
	ldi timerlow, 250
	ldi timerhigh, 0 ; wait 250 usec
	rcall wait_usec
ret


;;;;;;;;;;;;;;;;;;;;;;;;;;;;Delay Functions;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
wait_msec:
	push r24
	push r25
	ldi r24, low(999)
	ldi r25, high(999)
	rcall wait_usec
	pop r25
	pop r24
	nop
	nop
	sbiw r24, 1
	brne wait_msec
ret

wait_usec:
	sbiw r24,1 ; 2 cycles (2/16 usec)
	call delay_8cycles ; 4+8=12 cycles
	brne wait_usec ; 1 or 2 cycles
ret

delay_8cycles:
	nop
	nop
	nop
	nop
ret