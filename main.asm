stm8/

	#include "mapping.inc"
	#include "stm8s103f.inc"
	
pointerX MACRO first
	ldw X,first
	MEND
pointerY MACRO first
	ldw Y,first
	MEND	
	


	segment byte at 100 'ram1'
buffer1 ds.b
buffer2 ds.b
buffer3 ds.b
buffer4 ds.b
buffer5 ds.b
buffer6 ds.b
buffer7 ds.b
buffer8 ds.b
buffer9 ds.b
buffer10 ds.b
buffer11 ds.b
buffer12 ds.b
buffer13 ds.b	; remainder byte 0 (LSB)
buffer14 ds.b	; remainder byte 1
buffer15 ds.b	; remainder byte 2
buffer16 ds.b	; remainder byte 3 (MSB)
buffer17 ds.b	; loop counter
captureH ds.b
captureL ds.b	
captureHS ds.b
captureLS ds.b
capture_state ds.b	
nibble1  ds.b
data	 ds.b
address  ds.b
signbit  ds.b
temp1    ds.b
result4  ds.b
result3  ds.b
result2  ds.b
result1  ds.b
buffers  ds.b 23	
	
	segment 'rom'
main.l
	; initialize SP
	ldw X,#stack_end
	ldw SP,X

	#ifdef RAM0	
	; clear RAM0
ram0_start.b EQU $ram0_segment_start
ram0_end.b EQU $ram0_segment_end
	ldw X,#ram0_start
clear_ram0.l
	clr (X)
	incw X
	cpw X,#ram0_end	
	jrule clear_ram0
	#endif

	#ifdef RAM1
	; clear RAM1
ram1_start.w EQU $ram1_segment_start
ram1_end.w EQU $ram1_segment_end	
	ldw X,#ram1_start
clear_ram1.l
	clr (X)
	incw X
	cpw X,#ram1_end	
	jrule clear_ram1
	#endif

	; clear stack
stack_start.w EQU $stack_segment_start
stack_end.w EQU $stack_segment_end
	ldw X,#stack_start
clear_stack.l
	clr (X)
	incw X
	cpw X,#stack_end	
	jrule clear_stack




main_loop.l
	mov CLK_CKDIVR,#$00	; cpu clock no divisor = 16mhz
	bset PD_DDR,#3		; set PD3 as output
	bset PD_CR1,#3		; set PD3 as pushpull
	bres PC_DDR,#4		; clear PC4 DDR to set as input , Timer1 channel4
	
uart_setup:
	;UART1_CK PD4
	;UART1_TX PD5
	;UART1_RX PD6
	ld a,#$03			  ;$0683 = 9600 ,$008B = 115200, 
	ld UART1_BRR2,a		  ; write BRR2 firdt
	ld a,#$68
	ld UART1_BRR1,a		  ; write BRR1 next
	bset UART1_CR2,#3	  ; enable TX
	bset UART1_CR2,#2	  ; enable RX
timer_setup
	bres TIM1_CR1,#0	; disable timer
	bset TIM1_CR1,#3	; enable OPM
	bres TIM1_SR1,#0	; clear update interrupt flag
	bset TIM1_CCMR4,#0	; write 01 in CC4S bits 1:0 to enable input channel 4
	bres TIM1_CCER2,#5	; edge polarity high = 0
	bset TIM1_CCER2,#4	; enable input capture
	bset TIM1_IER,#0	; enable update interrupt
	bset TIM1_IER,#4	; enable capture compare chaneel 4 interrupt
	ld a,#$00
	ld TIM1_PSCRH,a		; msb 1st then lsb of prescaler
	ld a,#$ff
	ld TIM1_PSCRL,a 	; prescaler 255 =16000000/255 == 62500cycles in 1 sec
	ld a,#$f4
	ld TIM1_ARRH,a		; load auto repeat register with 62500= F424, high =0xf4 ,MSB write first
	ld a,#$24
	ld TIM1_ARRL,a		; load auto repeat register with 62500= F424, low =0x24 , LSB written after MSB
	bset TIM1_CR1,#0	; enable timer 
	RIM					; enable global interrupt

	pointerX #string
stringloop:
	ld a,(X)
	incw X
	cp a,#$00
	jreq exitstringloop
	ld data,a
	call UART_TX
	jp stringloop
exitstringloop:
	nop


wait:
	bset TIM1_CCER2,#4	; enable input capture ,restart capture4
	bset TIM1_CR1,#0	; enable timer, restart timer1 as we are in One pulse mode
	jp wait
	
	
	
A32bit_subtraction:	
	ld a,buffer8
	sub a,buffer4
	ld result1,a
	ld a,buffer7
	sbc a,buffer3
	ld result2,a
	ld a,buffer6
	sbc a,buffer2
	ld result3,a
	ld a,buffer5
	sbc a,buffer1
	ld result4,a
	JRULT load_signbit_register
	clr signbit
	ret
load_signbit_register
	mov signbit,#1
	ret
	
	

A32bit_subtraction1:	
	ld a,result1
	sub a,buffer4
	ld result1,a
	ld a,result2
	sbc a,buffer3
	ld result2,a
	ld a,result3
	sbc a,buffer2
	ld result3,a
	ld a,result4
	sbc a,buffer1
	ld result4,a
	JRULT load_signbit_register1
	clr signbit
	ret
load_signbit_register1
	mov signbit,#1
	ret


UART_TX:
	ld a,data
	ld UART1_DR,a
TC_FLAG:
	btjf UART1_SR,#6 ,TC_FLAG
	ret
	
bin_to_ascii:
	;jra wait
	;ldw x,#$1234
	;ldw buffer5,X
	;ldw x,#$0056
	;ldw buffer3,x
	ldw x,buffer16
	ldw data,x			; result 16bit word stored in buffer5 + buffer6 in data and address registers
	ld a,buffer15		; result MSB in buffer4 stored in nibble register sram, concecutive bytes
	ld nibble1,a		; result MSB in buffer4 stored in nibble register sram, concecutive bytes
	;ldw x,buffer5
	;ldw data,x			; result 16bit word stored in buffer5 + buffer6 in data and address registers
	;ld a,buffer4		; result MSB in buffer4 stored in nibble register sram, concecutive bytes
	;ld nibble1,a		; result MSB in buffer4 stored in nibble register sram, concecutive bytes
	clr buffer1			; clear sram registers for bin_to_ascii calculations
	clr buffer2			; clear sram registers for bin_to_ascii calculations
	clr buffer3			; clear sram registers for bin_to_ascii calculations
	clr buffer4			; clear sram registers for bin_to_ascii calculations
	clr buffer5			; clear sram registers for bin_to_ascii calculations
	clr buffer6			; clear sram registers for bin_to_ascii calculations
	clr buffer7			; clear sram registers for bin_to_ascii calculations
	clr buffer8			; clear sram registers for bin_to_ascii calculations
	clr result4			; clear sram registers for bin_to_ascii calculations
	clr result3			; clear sram registers for bin_to_ascii calculations
	clr result2			; clear sram registers for bin_to_ascii calculations
	clr result1			; clear sram registers for bin_to_ascii calculations
onecrore:
	ldw x,#$9680		; load x with low word of 10,000,000
	ldw buffer3,x		; store in buffer3 and buffer4
	ldw x,#$0098		; load x with high word of 10,000,000
	ldw buffer1,x		; store in buffer1 and buffer2,(buffer1,2,3,4 used for holding test value)
	mov buffer6,nibble1	; mov MSB of result in nibble1 to buffer6 (buffer5,6,7,8 used for holding result)
	ldw x,data			; load result word (LSB1,LSB0) to data & address register in sran (concecutive) 
	ldw buffer7,x		; load result word (LSB1,LSB0) to data & address register in sran (concecutive)
	call A32bit_subtraction		; call 32 bit subtraction routine, buffer5,6,7,8 - buffer1,2,3,4)
	inc temp1			; increase temp register to count how many 1 crrore in result
	ld a,signbit		; copy signbit register contents to accumulator
	jreq onecrore		; if signbit register is 0 (previous subtraction didnt result in negative) branch onecrore label
	dec temp1			; if negative value in subtraction , decrease temp register (we dont count)
revert_result0:	
	ld a,result1		; laod A with LSB of sutracted result1
	add a,buffer4		; add A with LSB0 of value subtracted. we reverse the result to pre negative value
	ld result1,a		; rectified LSB0 stored back in result1 
	ld a,result2		; laod A with LSB1 of sutracted result2
	adc a,buffer3		; add A with LSB1 of value subtracted. we reverse the result to pre negative value
	ld result2,a		; rectified LSB1 stored back in result2
	ld a,result3		; laod A with LSB2 of sutracted result3
	adc a,buffer2		; add A with LSB2 of value subtracted. we reverse the result to pre negative value
	ld result3,a		; rectified LSB2 stored back in result3 
	ld a,result4		; laod A with MSB of sutracted result4
	adc a,buffer1		; add A with MSB of value subtracted. we reverse the result to pre negative value
	ld result4,a		; rectified MSB stored back in result3 
	ld a,#$30			; ascii 0 loaded in A
	add a,temp1			; add temp1 (contains how many decimal places) to ascii 0 to get ascii value of poaition
	ld buffers ,a		; store result of ascii conversion of MSB position in buffers register SRAM
	clr temp1			; clear temp1 for next decimal position calculation
tenlakh:
	ldw x,#$4240
	ldw buffer3,x
	ldw x,#$000f
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq tenlakh
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 1} ,a	
	clr temp1
onelakh:
	ldw x,#$86A0
	ldw buffer3,x
	ldw x,#$0001
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq onelakh
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 2} ,a
	clr temp1
tenthousand:
	ldw x,#$2710
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq tenthousand
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 3} ,a
	clr temp1
thousand:
	ldw x,#$3e8
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq thousand
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 4} ,a
	ld a,#'.'
	ld {buffers + 5} ,a
	clr temp1
hundred:
	ldw x,#$0064
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq hundred
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 6} ,a
	clr temp1
ten:
	ldw x,#$000A
	ldw buffer3,x
	ldw x,#$0000
	ldw buffer1,x
	mov buffer6,nibble1
	ldw x,data
	ldw buffer7,x
	call A32bit_subtraction1
	inc temp1
	ld a,signbit
	jreq ten
	dec temp1
	
	ld a,result1
	add a,buffer4
	ld result1,a		; result LSB1
	ld a,result2
	adc a,buffer3
	ld result2,a		; result LSB2
	ld a,result3
	adc a,buffer2
	ld result3,a		; result LSB3
	ld a,result4
	adc a,buffer1
	ld result4,a		; result MSB
	ld a,#$30			; ascii 0
	add a,temp1
	ld {buffers + 7} ,a
	clr temp1				
	ld a,#$30			; ascii 0
	add a,result1
	ld {buffers + 8},a
	ld a,#'\n'			; new line
	ld {buffers + 9},a
	ld a,#'\n'			; new line
	ld {buffers + 10},a
	ld a,#'\r'			; carriage return
	ld {buffers + 11},a
	
	clr buffer1			; clear sram registers for bin_to_ascii calculations
	clr buffer2			; clear sram registers for bin_to_ascii calculations
	clr buffer3			; clear sram registers for bin_to_ascii calculations
	clr buffer4			; clear sram registers for bin_to_ascii calculations
	clr buffer5			; clear sram registers for bin_to_ascii calculations
	clr buffer6			; clear sram registers for bin_to_ascii calculations
	clr buffer7			; clear sram registers for bin_to_ascii calculations
	clr buffer8			; clear sram registers for bin_to_ascii calculations
	clr result4			; clear sram registers for bin_to_ascii calculations
	clr result3			; clear sram registers for bin_to_ascii calculations
	clr result2			; clear sram registers for bin_to_ascii calculations
	clr result1			; clear sram registers for bin_to_ascii calculations
	ret

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;32bit/32bit Unsigned Division
; tested works fine
; Register Variables
;  Call:  var1[3:0] = dividend (0x00000000..0xffffffff)
;         var2[3:0] = divisor (0x00000001..0x7fffffff)
;         mod[3:0]  = <don't care>
;         lc        = <don't care> (high register must be allocated)
;
;  Result:var1[3:0] = var1[3:0] / var2[3:0]
;         var2[3:0] = <not changed>
;         mod[3:0]  = var1[3:0] % var2[3:0]
;         lc        = 0
;
; Size  = 26 words
; Clock = 549..677 cycles (+ret)
; Stack = 0 bytes
;buffer1 db.s	   ;var13
;buffer2 db.s	   ;var12
;buffer3 db.s	   ;var11
;buffer4 db.s	   ;var10
;buffer5 db.s	   ;var23
;buffer6 db.s	   ;var22
;buffer7 db.s	   ;var21
;buffer8 db.s	   ;var20
;buffer9 db.s      ;modulo3
;buffer10 db.s     ;modulo2
;buffer11 db.s     ;modulo1
;buffer12 db.s     ;modulo0
;buffer13 db.s     ;counter lc
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;initialize variables
div32u:
	clr buffer12      	;modulo0
	clr buffer11      	;modulo1
	clr buffer10      	;modulo2
	clr buffer9       	;modulo3
	ld a,#32
	ld buffer13,a	  	;loop counter loaded with 32
calculating_loop:
;var1 = var1 << 1
	sll buffer4	  		;var10	
	rlc buffer3	  		;var12
	rlc buffer2 		;var11 
	rlc buffer1	    	;var10
;mod = mod << 1 + carry;
	rlc buffer12      	;modulo0
	rlc buffer11      	;modulo1
	rlc buffer10      	;modulo2
	rlc buffer9       	;modulo3
;if (mod => var2) {mod -= var2; var1++;}
; this block is used to do multibyte compare with carry,stm8 assembly has no CPC opcode
; CP & CPC is equelant to SUB&SBC. Take a copy of buffer5678 and store in buffer14,15,16,17
; as SUB&SBC modifies the values whereas CP doesnt.We modify only the copy in buffer14,15,16,17
; original buffer5,6,7,8 remains unchanged for other operations
compare_proc:
	call copy_buffer5678		; loading the buffer8,7,6,5 to buffer17,16,15,14 for subtrating. we dont have CPC
	ld a,buffer12 
	sub a,buffer17	   	;cp		mod0,var20
	ld a,buffer11
	sbc a,buffer16	   	;cpc	mod1,var21
	ld a,buffer10
	sbc a,buffer15	   	;cpc	mod2,var22
	ld a,buffer9
	sbc a,buffer14	   	;cpc	mod3,var23
	clr buffer17		; clear for net subtraction in next loop
	clr buffer16		; clear for net subtraction in next loop
	clr buffer15		; clear for net subtraction in next loop
	clr buffer14		; clear for net subtraction in next loop
	jrc decrease_counter
	inc buffer4	   		;var10
	ld a,buffer12      	;modulo0
	sub a,buffer8	   	;sub	mod0,var20
	ld buffer12,a
	ld a,buffer11      	;modulo1
	sbc a,buffer7	   	;sbc	mod1,var21
	ld buffer11,a	
	ld a,buffer10      	;modulo2
	sbc a,buffer6	   	;sbc	mod2,var22
	ld buffer10,a
	ld a,buffer9       	;modulo3
	sbc a,buffer5	   	;sbc	mod3,var23
	ld buffer9,a
decrease_counter:
	dec buffer13	   	;counter
	jrne calculating_loop
	ret					;answer in buffer1,2,3,4 msb to lsb

copy_buffer5678:
	ldw x,buffer7		;copy buffer7,8 to buffer16,17
	ldw buffer16,x
	ldw x,buffer5		;copy buffer5,6 to buffer14,15
	ldw buffer14,x
	ret

move_buffer1to4_buffer14to17:
	ldw x,buffer3		; store lower word of result in buffer16,17
	ldw buffer16,X
	ldw x,buffer1		; store higher word of result in buffer14,15
	ldw buffer14,X
	ret


string:
	  dc.B " Hello world!" ,'\n','\n','\r',0

	
	
	interrupt TIM1_ISR
TIM1_ISR
	
	bres TIM1_SR1,#0 ; clear interrupt flag
	bcpl PD_ODR,#3	;toggle led pin
	mov capture_state,#0
	bres TIM1_CCER2,#4
	
	iret


	interrupt TIM1_CAPTURE_ISR
TIM1_CAPTURE_ISR
	ld a,capture_state
	cp a,#0
	jrne second_capture
	ld a,TIM1_CCR4H ; clear TIM1_SR1 interrupt flag by reading TIM1_CCR4
	ld captureH,a
	ld a,TIM1_CCR4L ; clear TIM1_SR1 interrupt flag by reading TIM1_CCR4
	ld captureL,a
	ld a,#1
	ld capture_state,a
	iret
second_capture:
	ld a,TIM1_CCR4H ; clear TIM1_SR1 interrupt flag by reading TIM1_CCR4
	ld captureHS,a
	ld a,TIM1_CCR4L ; clear TIM1_SR1 interrupt flag by reading TIM1_CCR4
	ld captureLS,a
	ld a,#0
	ld capture_state,a
calculate_transmit:	
	ldw X,captureHS	; load word from capturHS & captureLS (2nd capture)
	subw X,captureH	; subtract captureH & captureL from capturHS & captureLS
	;ldw buffer1,X	; store the difference between 1st & 2nd capture in buffer1 + buffer2
	ldw buffer14,X	; store the difference between 1st & 2nd capture in buffer14 + buffer15
	ld a,#16		; 1000ms/62500cycles = 0.016 , fixed point math mul 16,put decimal point 3 place left
	mul X,a			; XL mul a , 16bbit result stored in X
	;ldw buffer5,X	; store above result in buffer5 and buffer6 (lsb)
	ldw buffer16,X	; store above result in buffer16 and buffer17 (lsb)
	ldw X,buffer14	; load another copy of capture result from buffer1 and buffer14
	SRLW X			; shift word in x right 1
	SRLW X			; shift word in x right 2
	SRLW X			; shift word in x right 3
	SRLW X			; shift word in x right 4
	SRLW X			; shift word in x right 5
	SRLW X			; shift word in x right 6
	SRLW X			; shift word in x right 7
	SRLW X			; shift word in x right 8
	mul x,a			; multiply MSB with 16
	ldw buffer14,x	; store result word in buffer14 and buffer15
	ld a,xl			; load A LSB of current result
	add a,buffer16	; add LSB with MSB of previous result
	ld buffer16,a	; store added result in buffer16
	ld a,buffer14	; copy buffer3 to A
	adc a,#0		; add with carry A
	ld buffer15,a	; answer (time in ms) in MSB buffer15,buffer16,buffer17 LSB max 24 byte 16bit x 8bit multiplication
	clr buffer14	; clear top byte in case of left over values there
	;;;;;;;;;;;;;;;;;
	
	ldw X,buffer16		; load X with lsb1 & lsb in buffer5 and buffer6
	ldw buffer3,X		; store word in X to buffer2 and buffer3  (DIVIDEND LSB)
	ldw X,buffer14		; load A with msb of result in buffer4 (buffer 4 , 5, 6)
	ldw buffer1,X		; store msb of result in buffer , now result in biffer1,buffer2,buffer3
						; DIVIDEND MSB , DIVIDEND IN buffer1,2,3,4
	ldw X,#$03e8		; low word of 1000 DIVISOR
	ldw buffer7,X		; store in buffer2 and buffer3
	ldw x,#$0000		; load a with msb WORD of 1000 24bit calculation
	ldw buffer5,x		; store A in buffer1 , numerator in buffer1,2,3
	call div32u			; call division routine for result/1000, result in buffer1,2,3,4
	
	call move_buffer1to4_buffer14to17
	
	ldw X,#$4240		; low byte of 1000000
	ldw buffer3,X		; store in buffer3 and buffer4
	ldw x,#$000f		; load x with msb of 1000000. 24bit calculation
	ldw buffer1,x		; store X in buffer1 , numerator in buffer1,2,3,4
	
	ldw X,buffer16		; load X with lsb1 & lsb in buffer16 and buffer17
	ldw buffer7,X		; store word in X to buffer3 and buffer4  (DIVISOR LSB)
	ldw X,buffer14		; load X with msb of result in buffer14 (buffer15 
	ldw buffer5,X		; store msb of result in buffer1,2 , (DIVISOR MSB)
	call div32u			; call division routine for result/1000, result in buffer1,2,3,4

	call move_buffer1to4_buffer14to17

	
  ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	
	call bin_to_ascii		; procedure to convert binary value to ASCII , values in buffer4,5,6
	pointerX #buffers   	; point X to buffers register , start point of ascii value storage, ascii values in buffers
	mov temp1,#12			; temp1 as counter, 11 values to be printed
TX_LOOP:
	ld a,(x)				; load A with value of string pointed by X
	ld data,a				; copy yte in A to data register
	call UART_TX			; call UART transmit subroutine
	incw X					; increase pointer X
	dec temp1				; decrease temp1 counter value
	jrne TX_LOOP			; loop to TX_LOOP label till temp1 is above 0
	
	iret



	
	
	

	interrupt NonHandledInterrupt
NonHandledInterrupt.l
	iret

	segment 'vectit'
	dc.l {$82000000+main}					; reset
	dc.l {$82000000+NonHandledInterrupt}	; trap
	dc.l {$82000000+NonHandledInterrupt}	; irq0
	dc.l {$82000000+NonHandledInterrupt}	; irq1
	dc.l {$82000000+NonHandledInterrupt}	; irq2
	dc.l {$82000000+NonHandledInterrupt}	; irq3
	dc.l {$82000000+NonHandledInterrupt}	; irq4
	dc.l {$82000000+NonHandledInterrupt}	; irq5
	dc.l {$82000000+NonHandledInterrupt}	; irq6
	dc.l {$82000000+NonHandledInterrupt}	; irq7
	dc.l {$82000000+NonHandledInterrupt}	; irq8
	dc.l {$82000000+NonHandledInterrupt}	; irq9
	dc.l {$82000000+NonHandledInterrupt}	; irq10
	dc.l {$82000000+TIM1_ISR}				; irq11 overflow interrupt
	dc.l {$82000000+TIM1_CAPTURE_ISR}		; irq12 timer1 capture/compare
	dc.l {$82000000+NonHandledInterrupt}	; irq13
	dc.l {$82000000+NonHandledInterrupt}	; irq14
	dc.l {$82000000+NonHandledInterrupt}	; irq15
	dc.l {$82000000+NonHandledInterrupt}	; irq16
	dc.l {$82000000+NonHandledInterrupt}	; irq17
	dc.l {$82000000+NonHandledInterrupt}	; irq18
	dc.l {$82000000+NonHandledInterrupt}	; irq19
	dc.l {$82000000+NonHandledInterrupt}	; irq20
	dc.l {$82000000+NonHandledInterrupt}	; irq21
	dc.l {$82000000+NonHandledInterrupt}	; irq22
	dc.l {$82000000+NonHandledInterrupt}	; irq23
	dc.l {$82000000+NonHandledInterrupt}	; irq24
	dc.l {$82000000+NonHandledInterrupt}	; irq25
	dc.l {$82000000+NonHandledInterrupt}	; irq26
	dc.l {$82000000+NonHandledInterrupt}	; irq27
	dc.l {$82000000+NonHandledInterrupt}	; irq28
	dc.l {$82000000+NonHandledInterrupt}	; irq29

	end
	