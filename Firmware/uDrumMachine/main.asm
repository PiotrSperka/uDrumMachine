;
; uDrumMachine.asm
;
; Created: 19.01.2018 19:08:52
; Author : Piotrek
;
; AVR works @ 20MHz -> 1 clock == 50ns
; Base sampling is about 30kHz == 33.3us which is about 666 clocks
; 8-bit DAC is built using PWM and low pass filter
;
; Each pattern consists of 16 steps, song consists of up to 256 patterns
; Device can save up to 256 patterns and 256 songs
; There are 8 instruments, and device has 8 voice poliphony
;
; Samples in EEPROM are mixed like this (Voice/Sample): V1S1|V2S1|...|V8VS1|V1S2|V2S2|...
; 25LC1024 EEPROM is used
;
; Version 0.0.1-20180119

; ----------------------------------
; -------- GLOBAL DEFINES ----------
; ----------------------------------
#define DD_MOSI		3
#define DD_SCK		5
#define DDR_SPI		DDRB

#define EE_CS		2
#define EE_DDR		DDRB
#define EE_PORT		PORTB

#define EE_READ		0b00000011
#define EE_WRITE	0b00000010
#define EE_WREN		0b00000110
#define EE_WRDI		0b00000100
#define EE_RDSR		0b00000101
#define EE_WRSR		0b00000001
#define EE_PE		0b01000010
#define EE_SE		0b11011000
#define EE_CE		0b11000111
#define EE_RDID		0b10101011
#define EE_DPD		0b10111001

#define LCD_DDR		DDRC
#define LCD_PORT	PORTC
#define LCD_RS		0
#define LCD_RW		1
#define	LCD_E		2

#define DATA_DDR	DDRD
#define DATA_PORT	PORTD
#define DATA_PIN	PIND

; RELATIVE MEMORY MAP:
; 0x0000 - 0x000F - Current pattern (16 bytes)
; 0x0010 - 0x004F - Sample buffer A (64 bytes => 8 voices * 8 samples) @ 30kHz buffer lasts for 266.66us (~5300 clks)
; 0x0050 - 0x008F - Sample buffer B (64 bytes => 8 voices * 8 samples)

#define RAM_START	0x0100 ; First address of RAM memory
#define BUFA_START	0x0010 ; Address of first sound buffer element relative to RAM_START
#define BUFB_START	0x0050 ; Address of first sound buffer element relative to RAM_START

; 'RESERVED' REGISTERS:
; R2 - Current pattern step
; R3 - Current pattern step number
; R4 - Next EEPROM sample address (MSB)
; R5 - Next EEPROM sample address (LSB)
; R6 - Buffer position (to check if there is a need to switch buffers)
; R7 - Flags register A: Update EEPROM address|Fill buffer A|Fill buffer B|Playing samples|Current sample buffer|X|X|Read next pattern
; R8 - Flags register B: X|X|X|X|X|X|X|X
; Y  - Sample buffer pointer
;

.MACRO LCM ; Load Constant from Memory
	ldi			ZH, high(2 * @0)
	ldi			ZL, low(2 * @0)
.ENDMACRO

; ----------------------------------
; -------- INT VECTORS -------------
; ----------------------------------
.org 0x0000			; RESET
	rjmp	RESET
.org 0x0009			; TIM2_OVF
	rjmp	SEND_SAMPLE
.org 0x000D			; TIM1_OVF
	rjmp	NEXT_BEAT

; ----------------------------------
; -------- INT ROUTINES ------------
; ----------------------------------
.org 0x0020
SEND_SAMPLE:
	; This routine really needs to be highly optimized
	; We don't use call do DAC_SEND because of optimization
	; 'Normal' operation takes 113 clocks
	; 'Extended' operation (every 8th call) takes XXX clocks
	; For now this routine takes about XX% of processor time
	push	r16				; 2 clocks
	push	r17				; 2 clocks
	push	r18				; 2 clocks
	push	r19				; 2 clocks
	rcall	MIX_SAMPLE		; 89 clocks
	out		OCR0A, r1		; 1 clk
	out		OCR0B, r1		; 1 clk
	dec		r6				; 1 clk, decrement left elements in buffer
	brne	Send_sample_exit	; 1/2 clks, skip if there is a need to switch buffers
	; Switch buffers here and prepare all necessary stuff
Send_sample_switch_buffers:
	ldi		r16, 8			; 1 clock, set new buffer counter
	mov		r6, r16			; 1 clock
	; Check which buffer needs to be set, write Y register and necessary flags
Send_sample_set_bufA:
	
	rjmp	Send_sample_exit ; 2 clocks
Send_sample_set_bufB:

Send_sample_exit:
	pop		r19				; 2 clocks
	pop		r18				; 2 clocks
	pop		r17				; 2 clocks
	pop		r16				; 2 clocks
	reti					; 4 clocks

NEXT_BEAT:
	; Load next step in R2, Check if there is a need to load new pattern, Zero the sample pointers, Set the sample EEPROM pointer, Read data
	reti			; 4 clocks

; ----------------------------------
; -------- MAIN ROUTINES -----------
; ----------------------------------
RESET:
    ldi		r16, 0xFF	;Load SP
	out		SPL, r16
	ldi		r16, 0x04
	out		SPH, r16

	; Pin configuration
	; PORTB
	;ldi		r16, (1 << EE_CS)
	;out		DDRB, r16
	;out		PORTB, r16 ; Set EE's CS

	; PORTD - DataBus - output
	ldi			r16, 0xFF
	out			DDRD, r16
	eor			r16, r16
	out			PORTD, r16

	; PORTC - Address decoder + LCD - Output
	ldi			r16, 0x3F
	out			DDRC, r16
	eor			r16, r16
	out			PORTC, r16

	; Timers configuration
	; Timer0 as PWM

	;in		r16, DDRD
	;ori		r16, (1 << PD5)|(1 << PD6)
	;out		DDRD, r16

	;ldi		r16, (1 << COM0A1)|(1 << COM0B1)|(1 << WGM00)|(1 << WGM01)
	;out		TCCR0A, r16
	;ldi		r16, (1 << CS02)
	;out		TCCR0B, r16
	;ldi		r16, 0x7F
	;out		OCR0A, r16
	;out		OCR0B, r16

	rcall	DELAY_50MS ; Wait for LCD startup

	; Initialization
	rcall	LCD_INIT
	lcm		TXT_NAME
	rcall	LCD_STRING
	ldi		r16, 0b11000000
	rcall	LCD_CMD
	lcm		TXT_VER
	rcall	LCD_STRING
	;rcall	SPI_INIT

	; Prepare all necessary registers for playing....

	sei				; interrupt enable

MAIN_LOOP:
	; Remember to check for new buffer request every 100us or so
	; Check if there is a need to read song or pattern
	; Check if there is a need to read new buffer
	; Check keyboard
	; Refresh LEDs
	; Refresh LCD

    rjmp	MAIN_LOOP

; ----------------------------------
; -------- MUSIC ROUTINES ----------
; ----------------------------------
; This routine really needs to be highly optimized
; Provide pattern step in r2, address of sample of first voice in (Y)
; Corrupts r0, r1, r16, r17, r18, r19
; Returns result in r1
; 9 clks * 8 + 5 clks + 9 clks = 86 clks
MIX_SAMPLE:
	clr		r0			; 1 clk
	clr		r1			; 1 clk
	ldi		r16, 8		; 1 clk
	clr		r18			; 1 clk, This is used as 'zero' register
	mov		r19, r2		; 1 clk
Mix_Sample_Loop:
	ror		r19			; 1 clk
	brcc	Mix_Sample_Next ; 1/2 clk
	ld		r17, Y+		; 2 clk
	add		r1, r17		; 1 clk
	adc		r0, r18		; 1 clk
Mix_Sample_Next:
	dec		r16			; 1 clk
	brne	Mix_Sample_Loop	; 1/2 clk
	; Divide result by 8 to prevent clipping
	ror		r0			; 1 clk
	ror		r1			; 1 clk, div by 2
	ror		r0			; 1 clk
	ror		r1			; 1 clk, div by 4
	ror		r0			; 1 clk
	ror		r1			; 1 clk, div by 8
	ret					; 4 clk

; ----------------------------------
; -------- SPI ROUTINES ------------
; ----------------------------------
; SPI init, corrupts r16
SPI_INIT:
	; Set MOSI and SCK output, all others input
	in		r16, DDR_SPI
	andi	r16, (1<<DD_MOSI)|(1<<DD_SCK) ; TODO: PRESCALE!!!
	out		DDR_SPI, r16
	; Enable SPI, Master, set clock rate fck/2
	ldi		r16, (1<<SPE)|(1<<MSTR)
	out		SPCR, r16
	ldi		r16, (1<<SPI2X)
	out		SPSR, r16
	ret

; Transmits byte from r16 and receives byte to r16
; At 10 MHz SPI it takes about 800ns + few clocks (~20 clks @ 20MHz)
SPI_SEND:
	; Start transmission of data (r16)
	out		SPDR, r16		; 1 clk
Spi_Wait_Transmit:
	; Wait for transmission complete, TODO: use interrupt?
	in		r16, SPSR		; 1 clk
	sbrs	r16, SPIF		; 1/2 clk
	rjmp	Spi_Wait_Transmit ; 2 clk
	in		r16, SPDR		; 1 clk
	ret						; 4 clk

; ----------------------------------
; -------- EXT. EEPROM ROUTINES ----
; ----------------------------------
; Set address to read, provide address in r17(MSB), r18, r19(LSB), corrupts r16
; You need to manage CS line by yourself!
EEP_READ_ADDR_SET:
	ldi		r16, EE_READ	; 1 clk
	rcall	SPI_SEND		; 3 + X clk
	mov		r16, r17		; 1 clk
	rcall	SPI_SEND		; 3 + X clk
	mov		r16, r18		; 1 clk
	rcall	SPI_SEND		; 3 + X clk
	mov		r16, r19		; 1 clk
	rcall	SPI_SEND		; 3 + X clk
	ret						; 4 clk

; Read eeprom to ram. Provide ram address in X register and byte count in r17
; You need to call EEP_READ_ADDR_SET first !!!
; You need to manage CS line by yourself!
; Optimized for speed, doesn't use SPI routines
; Takes about 1us, 20 clks per byte (20MHz, 1MHz SPI)
EEP_READ:
	clr		r16				; 1 clk
	; Start transmission of data (r16)
	out		SPDR, r16		; 1 clk
Eep_Spi_Wait_Transmit:
	; Wait for transmission complete
	in		r16, SPSR		; 1 clk
	sbrs	r16, SPIF		; 1/2/3 clk
	rjmp	Eep_Spi_Wait_Transmit ; 2 clk
	in		r16, SPDR		; 1 clk
	st		X+, r16			; 2 clk
	dec		r17				; 1 clk
	brne	EEP_READ		; 1/2 clk
	ret						; 4 clk


; ----------------------------------
; -------- DAC ROUTINES ------------
; ----------------------------------
; Sends to DAC value from r16 (to both channels for now...)
; This can be optimized by outputting OCR registers directly from mixer routine, without call
; 6 clks
DAC_SEND:
	out		OCR0A, r16		; 1 clk
	out		OCR0B, r16		; 1 clk
	ret						; 4 clk

; ----------------------------------
; -------- LED ROUTINES ------------
; ----------------------------------

; ----------------------------------
; -------- BUTTON ROUTINES ---------
; ----------------------------------

; ----------------------------------
; -------- LCD ROUTINES ------------
; ----------------------------------
LCD_INIT:
	push		r16
	push		r17
	push		r18
	push		r19

	ldi			r16, 0x04
	out			PORTC, r16 ; RS = 0, RW = 0, E = 1
	ldi			r16, 0x30
	out			PORTD, r16
	cbi			PORTC, 2 ; E = 0
	rcall		DELAY_5MS
	sbi			PORTC, 2 ; E = 1
	rcall		DELAY_100US
	cbi			PORTC, 2 ; E = 0
	rcall		DELAY_100US
	sbi			PORTC, 2 ; E = 1
	rcall		DELAY_100US
	cbi			PORTC, 2 ; E = 0
	rcall		DELAY_100US
	sbi			PORTC, 2 ; E = 1
	rcall		DELAY_100US

	ldi			r16, 0x38 ; 16ch per line, 5x8
	rcall		LCD_CMD

	ldi			r16, 0x08 ; Display shift to left
	rcall		LCD_CMD

	ldi			r16, 0x01 ; Display home
	rcall		LCD_CMD

	ldi			r16, 0x0C ; Display on, cursor off, blink off
	rcall		LCD_CMD

	ldi			r16, 0x06 ; Entry mode set
	rcall		LCD_CMD

	pop			r19
	pop			r18
	pop			r17
	pop			r16

	ret

LCD_WAIT:
	ldi			r17, 0x00
	out			DATA_DDR, r17 ; DataBus In
	ldi			r17, 0xFF
	out			DATA_PORT, r17 ; DataBus Pullup
	sbi			LCD_PORT, LCD_RW
	cbi			LCD_PORT, LCD_RS

	_lcd_busy_1:
	rcall		DELAY_500NS
	cbi			LCD_PORT, LCD_E ; E = 0
	rcall		DELAY_500NS
	sbi			LCD_PORT, LCD_E ; E = 1
	rcall		DELAY_500NS
	in			r17, DATA_PIN
	sbrc		r17, 7
	rjmp		_lcd_busy_1
	cbi			LCD_PORT, LCD_RW

	ret

LCD_CMD:
	push		r17
	push		r18

	ldi			r17, 0xFF
	out			DATA_DDR, r17 ; DataBus Out
	in			r17, LCD_PORT
	andi		r17, (~((1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_E)))
	ori			r17, (1 << LCD_E)
	out			LCD_PORT, r17

	out			DATA_PORT, r16
	cbi			LCD_PORT, LCD_E ; E = 0
	rcall		DELAY_500NS
	sbi			LCD_PORT, LCD_E ; E = 1

	rcall		LCD_WAIT
	rcall		DELAY_10US

	pop			r18
	pop			r17

	ret

LCD_DAT:
	push		r17
	push		r18

	ldi			r17, 0xFF
	out			DATA_DDR, r17 ; DataBus Out
	in			r17, LCD_PORT
	andi		r17, (~((1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_E)))
	ori			r17, ((1 << LCD_RS) | (1 << LCD_E))
	out			LCD_PORT, r17

	out			DATA_PORT, r16
	cbi			LCD_PORT, LCD_E ; E = 0
	rcall		DELAY_500NS
	sbi			LCD_PORT, LCD_E ; E = 1

	rcall		LCD_WAIT
	rcall		DELAY_10US

	pop			r18
	pop			r17

	ret

LCD_CLEAR:
	ret

LCD_POSITION:
	ret

LCD_CHAR:
	ret

LCD_STRING:
	push		r16

	_lcd_string_next:
	lpm			r16, Z+
	and			r16, r16
	breq		_lcd_string_end
	rcall		LCD_DAT
	rjmp		_lcd_string_next

	_lcd_string_end:
	pop			r16

	ret
	
; ----------------------------------
; -------- UI ROUTINES -------------
; ----------------------------------


; ----------------------------------
; -------- DELAY ROUTINES ----------
; ----------------------------------
DELAY_50MS:
    ldi  r18, 6
    ldi  r19, 19
    ldi  r20, 172
L0: dec  r20
    brne L0
    dec  r19
    brne L0
    dec  r18
    brne L0
    nop
	ret

DELAY_5MS:
    ldi  r18, 130
    ldi  r19, 220
L1: dec  r19
    brne L1
    dec  r18
    brne L1
	ret

DELAY_100US:
    ldi  r18, 3
    ldi  r19, 150
L2: dec  r19
    brne L2
    dec  r18
    brne L2
	ret

DELAY_10US:
    ldi  r18, 64
L3: dec  r18
    brne L3
    nop

DELAY_500NS:
	lpm
	ret

; ----------------------------------
; -------- TEXT CONSTANTS ----------
; ----------------------------------
TXT_NAME: .db " uDrum Machine  ", 0, 0
TXT_VER:  .db "    v1.0.0      ", 0, 0
