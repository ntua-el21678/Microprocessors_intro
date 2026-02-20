.include "m328pbdef.inc"  ; include device definitions

.org 0x00
.def A = r18
.def B = r19
.def C = r20
.def D = r21
.def F0 = r22
.def F1 = r23
.def counter = r24
.def temp = r25

rjmp reset

reset:
	ser r16
	out DDRD,r16

	ldi A,0x52
	ldi B,0x42
	ldi C,0x22
	ldi D,0x02
	ldi F0,0x00
	ldi F1,0x00
	ldi counter,6

	rcall loop

loop:
	mov F0,A
	com F0
	or F0,B
	mov temp,B
	com temp
	or temp,D
	and F0,temp
	com F0

	mov F1,A
	and F1,C
	mov temp,B
	and temp,D
	or F1,temp
	
	ldi temp,1
	add A,temp
	inc temp
	add B,temp
	inc temp
	add C,temp
	inc temp
	add D,temp

	lsl F1
	or F1,F0
	out PORTD,F1

	dec counter
	brne loop
	