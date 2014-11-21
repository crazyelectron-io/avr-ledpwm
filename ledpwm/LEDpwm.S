;*==================================================================================================*;
;*SYNOPSIS:																							*;
;*	Atmel AVR Library with PWM routines for LED driver.												*;
;*																									*;
;*VERSION HISTORY:																					*;
;*	20141109 v0.1	Initial test version.															*;
;*																									*;
;*DESCRIPTION:																						*;
;*	Routines for Atmel AVR 8-bit MCU's that drive a LED through variable PWM signal for dimming.	*;
;*	For more details see the include file "ledpwm.h".												*;
;*																									*;
;*	Fast PWM mode timer function of PORTB4 (OC1B) is used as LED driver. We use the system clock	*;
;*	from the MCU running at 8 MHz in asynchronuous clock mode with a prescaler; OCR1C, CS1[3:0] to	*;
;*	control the PWM frequency.																		*;
;*	The Fast PWM signal generation works like this:													*;
;*																									*;
;*              <--PWM frequency---> = (PCK/<prescaler>)/(OCR1C+1)									*;
;*              <duty cycle-> = OCRnB																*;
;*				+-----------+      +-----------+      +----											*;
;*		OCnB	|           |      |           |      |												*;
;*		--------+           +------+           +------+												*;
;*				0                 OCRnC																*;
;*																									*;
;*	The PWM output on OCnB (PORTB4) is high until the OCR1B value is reached, and low until the		*;
;*	timer reaches the top value OCRnC and wraps back to 0. Setting OCRnB to 0x00 gives DT=100%;		*;
;*  setting it to 0xFF will give DT=0%.																*;
;*																									*;
;*NOTES:																							*;
;*	1. It is assumed that all generic initialization, like stackpointer setup is done by the		*;
;*		calling program.																			*;
;*	2.	This library defines one interrupt vector (TIMERn); all other vectors are up to the			*;
;*		calling program.																			*;
;*	3.	Since we use the high speed (64 MHz) pheripheral clock, the supply voltage should not drop	*;
;*		below 2.7 volts.																			*;
;*																									*;
;*COPYRIGHT:																						*;
;*	(c)2014 by Ron Moerman, All Rights Reserved.													*;
;*																									*;
;*	This program comes with ABSOLUTELY NO WARRANTY.													*;
;*	This is free software, and you are welcome to redistribute it under certain conditions.			*;
;*	The program and its source code are published under the GNU General Public License (GPL).		*;
;*	See http://www.gnu.org/licenses/gpl-3.0.txt for details.										*;
;*																									*;
;*	$File: LEDpwm.S $																				*;
;*	$Revision: 0.1 $																				*;
;*	$ASM: AVR-GCC AS $																				*;
;*	$Author: Ron Moerman $																			*;
;*	$Email: ron@moerman.cc $																		*;
;*	$Date: Tuesday, November 09, 2014 20:56:23 UTC $												*;
;*==================================================================================================*;

;*==================================================================================================*;
;*                                   I N C L U D E   H E A D E R S                                  *;
;*==================================================================================================*/

#define __SFR_OFFSET 0

		.nolist
#include <avr/io.h>
#include <avr/timer.h>
#include <avr_macros.h>
		.list
//#include <ledpwm.h>


;*==================================================================================================*;
;*                          D E V I C E   D E P E N D E N T   D E F I N E S                         *;
;*==================================================================================================*;

#if defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__)||defined(__AVR_ATtiny4313__)
	#define PWM_DIV (0<<CS02|1<<CS01|1<<CS00)			//PWM Clock prescaler is PCK/64.
	#define PWM_MODE (1<<COM0A1|1<<COM0A0|1<<WGM1|1<<WGM0)	//Fast PWM: Set OC0A on Compare Match, Clear at TOP.
	#define PWM_TCCRA TCCR0A							//PWM T/C control register A.
	#define PWM_TCCRB TCCR0B							//PWM T/C control register B.
	#define PWM_OCR OCR0A

	#define PWM_DTPORT OCR0B ??
	#define PWM_FREQPORT OCR0C ??
	#define PWM_DIR_PORT PORTD ??
	#define PWM_DIR_DDR DDRD ??
	#define PWM_DIR_DDPIN DDD3 ??
	#define PWM_DIR_PIN PD3 ??
	#define PWM_OUTPORT PB3 ??
#elif defined (__AVR_ATtiny85__)||defined(__AVR_ATtiny45___)
// ...
#else
	#error "Only ATtinyX313(A)/ATtinyX8 supported (for now)."
#endif


;==========================================================================================
;                               F U S E S   &   P R O G R A M M I N G
;==========================================================================================
;	Fuse Bit Settings:
;	-----------------
;	1. Set Preserve EEPROM memort throug chip erase cycle (EESAVE)
;	2. Use internal RC oscillator at 8MHZ (CKSEL)
;	3. Serial Program downloading (SPI) enabled (SPIEN)
;	4. Brown-out detection level at VCC=2.7V [BODLEVEL)
;	5. Start-up time PWRDWN/RESET: 6 CK/14CK + 64ms (SUT)
;
;	Resulting Fuse Settings:
;LOW:	b7	CKDIV8		0	Don't divide clock by 8
;		b6	CKOUT		1	???
;		b5	SUT1		1	Select startup time/delay
;		b4	SUT0		0	(6 CK/14CK + 64ms)
;		b3	CKSEL3		0	Select clock source
;		b2	CKSEL2		0	(xxxxxxx)
;		b1	CKSEL1		1
;		b0	CKSEL0		0
;HIGH:	b7	RSTDISBL	0	External Reset enable
;		b6	DWEN		0	DebugWIRE Enable/Disable
;		b5	SPIEN		0	Enable Serial Program and Data Downloading
;		b4	WDTON		0	Watchdog Timer off
;		b3	EESAVE		1	EEPROM memory is preserved through the Chip Erase
;		b2	BODLEVEL2	1	Brown-out Detector trigger level
;		b1	BODLEVEL1	0	 (set at 2.7V)
;		b0	BODLEVEL0	1
;EXTND:	b0	SELFPRGEN	1	Self-programming Enable
;
;NOTE: A 1 means unprogrammed for the EEPROM, 0 means programmed.


;*==================================================================================================*;
;*                                         C O N S T A N T S                                        *;
;*==================================================================================================*;

;For a 4 kHz PWM frequency we use a CK/8 prescaler and a PWM_FREQPORT (Compare Match) value of 249:
;    fpwm = CK/8/(249+1) = 8000000/8/250 = 4000 = 4 kHz.
;
PWM_FREQ = 0xF9											;PWM frequency value 249 (set in PWM_FREQPORT)


;*==================================================================================================*;
;*                                L I N K E R   D E F I N I T I O N S                               *;
;*==================================================================================================*;

; Interrupt routines.
		.global TIMER0_OVF_vect							;Timer1 Overflow interrupt routine entrypoint.

; Make these library funtions externally accessible.
		.global ledpwm_init
		.global	ledpwm_dtcycle


;*==================================================================================================*;
;*                                  E E P R O M   V A R I A B L E S                                 *;
;*==================================================================================================*;
		.section .eeprom

ee_dtcycle:
		.byte	0										;Saved Duty Cycle value for PWM LED signal.


;*==================================================================================================*;
;*                           L I B R A R Y   L O C A L   R E G I S T E R S                          *;
;*==================================================================================================*;

		.equ	DTCR, 3									;Register holding the active duty cycle.


;*==================================================================================================*;
;*                                I N T E R R U P T   R O U T I N E S                               *;
;*==================================================================================================*;
		.section .text

;*--------------------------------------------------------------------------------------------------*;
;* TIMER1_OVF_vect: ISR triggered on Timer1 Overflow.												*;
;*--------------------------------------------------------------------------------------------------*;
;*DESCRIPTION:																						*;
;*	ISR triggered on Timer1 Overflow counter to control the PWM duty cycle.							*;
;*																									*;
;*INPUT:																							*;
;*	None.																							*;
;*																									*;
;*OUTPUT:																							*;
;*	None.																							*;
;*																									*;
;*REGISTER USAGE:																					*;
;*	R0 (SREG).																						*;
;*																									*;
;*LOCAL STACK USAGE:																				*;
;*	X bytes.																						*;
;*																									*;
;*NOTES:																							*;
;*	2.	Any error that occurs is added to the error buffer.											*;
;*--------------------------------------------------------------------------------------------------*;
TIMER1_OVF_vect:
		in		R0,SREG									;Save status register in R0. (1)
;
;		out		PWM_DTPORT,DTCR							;Set DT value.
		out		PWM_OCR,DTCR
;
		out		SREG,R0									;Restore status register and return.
		reti


;*==================================================================================================*;
;*                          L E D   P W M   L I B R A R Y   R O U T I N E S						    *;
;*==================================================================================================*;

;*--------------------------------------------------------------------------------------------------*;
;* ledpwm_init: Setup PWM signal on PWMOUT port.													*;
;*--------------------------------------------------------------------------------------------------*;
;*DESCRIPTION:																						*;
;*	Setup PWM signal on PWMOUT port. Generate hadware clock based PWM signal, using the 8-bit		*;
;*	Timer/Counter1 in asynchronous clocking mode (with the fast peripheral clock running at 64 MHz	*;
;*	as clock time base) to generate a PWM signal on PWMA, with OCR1C as PWM frequency settings		*;
;*	register and OCR1A as DT settings register. We use T/C1 because it has a seperate prescaler		*;
;*	running from PLL.																				*;
;*	Set PWMOUT to toggle on Compare Match (by setting COM1x1 and COM1x0 both to 1; see Table 12-1,	*;
;*	page 86 of the ATtiny85 datasheet). Set the T/C1 Prescaler Select bits to create a 64MHz/256 =	*;
;*	250kHz PWM clock source.																		*;
;*																									*;
;*INPUT REGISTERS:																					*;
;*	None.																							*;
;*																									*;
;*OUTPUT:																							*;
;*	None.																							*;
;*																									*;
;*REGISTERS CHANGED:																				*;
;*	R24.																							*;
;*																									*;
;*STACK USAGE:																						*;
;	X bytes (including call to this routine and called routines).									*;
;*																									*;
;*NOTES:																							*;
;*	1. This routine uses XX CPU cycles, including return to calling routine.						*;
;*--------------------------------------------------------------------------------------------------*;
		.func	ledpwm_init
ledpwm_init:
		push	XL										;Save used registers. (2/4)
#if (EEPROMEND > 256)
		push	XH
#endif
;Setup PWM signal on PWM_OUT.

;Generate hadware clock based PWM signal, using the 8-bit Timer/Counter0 in asynchronous clocking
; mode (with the fast peripheral clock running at 64 MHz as clock time base) to generate a PWM
; signal on PWMA, with OCR0C as PWM frequency settings register and OCR0A as Duty Cycle
; settings register. We use Timer/Counter1 because it has a seperate prescaler running from PLL.
;Set PWM_OUT to toggle on Compare Match (by setting COM0A1 and COM0A0 both to 1).
; Also, set the Timer/Counter0 Prescaler Select bits to create a 64MHz/64 = 1 MHz PWM clock source.
		ldi		R24,PWM_MODE|1<<WGM1|1<<WGM0 
		out		PWM_TCCRA,R24							;Set OC0A on Compare Match and clear at TOP.
		ldi		R24,1<<WGM2|PWM_DIV						;Set Fast PWM mode and prescaler to 64MHz/64.
		out		PWM_TCCRB.R24
;
; Set the PWM frequency and initial PWM duty cycle (DT) value.
		ldi		R24,PWM_FREQ
		out		PWM_FREQPORT,R24						;Write clock divider to set PWM frequency.
		ldi		XL,lo8(ee_dtcycle)
#if (EEPROMEND > 256)
		ldi		XH,hi8(ee_dtcycle)
#endif
		rcall	ee_read_byte							;Read to stored PWM DT cycle from EEPROM.
		out		PWM_DTPORT,R24							;Set saved or default duty cycle.
		mov		DTCR,R24								;Save DT value in 'duty cycle' register (DTCR).
;
; Set T/C1 in async mode: enable PLL, wait >100 us to stabilize, poll the PLOCK bit
;  until it is set, then set the PCKE bit in the PLLCSR register.
		ldi		R24,(1<<PLLE)
		out		PLLCSR,R24
		clr		R24
1:		inc		R24										;Delay a bit.
		brne	1b
2:		in		R24,PLLCSR								;Then wait until PLL locked.
		sbrs	R24,PLOCK
		rjmp	2b
; PLL is stabilized and locked, now set the PCKE bit to set PLL as T/C1 source.
		sbi		PLLCSR,PCKE
;
; Set PWMOUTPORT as output an the rest of PORTB as inputs with pull up.
		out		DDRB,ZEROR								;First, set all ports as input.
		nop												;  (for synchronization purposes)
		ldi		R24,(1<<PWM_OUTPORT)					;Then, set the PWM B port as output.
		out		DDRB,R24
		ldi		R24,0x3F								;Enable internal pull up for PORTB pins, except PWM A to
		out		PORTB,R24								;  ensure a defined level for the unused pins.
;
; Enable PWM B signal on PORTB4.
		ldi		R24,(1<<PWM1B)|(1<<COM1B1)|(1<<COM1B0)
		out		GTCCR,R24
;
; Set Timer/Counter0 Overflow Interrupt Enable bit (TOIE0) to enable the Timer/Counter0 Overflow Interrupt.
; The corresponding interrupt vector 0x004? is executed if an overlow in Timer/Counter0 occurs.
		ldi		R24,(1<<TOIE0)
		out		TIMSK,R24
		clc
#if (EEPROMEND > 256)
		pop		XH										;Restore registers and return. (6/8)
#endif
		pop		XL
		ret
		.endfunc


;*--------------------------------------------------------------------------------------------------*;
;* ledpwm_dtcycle: Update the PWM Duty Cycle value stored in EEPROM.								*;
;*--------------------------------------------------------------------------------------------------*;
;*DESCRIPTION:																						*;
;*	This routine updates the PWM Duty Cycle value that is kept in EEPROM for power failure recovery	*;
;*	and will be updated as changes in LED brightness are set.										*;
;*	To prevent wear of EEPROM memory, the EEPROM write routines check if the new value to be		*;
;*	written is changed, otherwise no write is performed.											*;
;*																									*;
;*INPUT REGISTERS:																					*;
;*	R24 = Duty Cycle value to store in EEPROM @ee_dtcycle and in DTCR.								*;
;*																									*;
;*OUTPUT:																							*;
;*	CF=0: OK.																						*;
;*																									*;
;*REGISTERS CHANGED:																				*;
;*	R24, DTCR.																						*;
;*																									*;
;*STACK USAGE:																						*;
;	X bytes (including call to this routine and called routines).									*;
;*																									*;
;*NOTES:																							*;
;*	1. This routine uses XXX CPU cycles, including return to calling routine.						*;
;*--------------------------------------------------------------------------------------------------*;
		.func	ledpwm_dtcycle
ledpwm_dtcycle:
		push	XL
#if (EEPROMEND > 256)
		push	XH
#endif
		mov		DTCR,R24								;Save DT cycle in DTCR register for easy reference.
; Write Duty Cycle value to EEPROM (if changed).
		ldi		XL,lo8(ee_dtcycle)
#if (EEPROMEND > 256)
		ldi		XH,hi8(ee_dtcycle)
#endif
		rcall	ee_write_byte
; Restore and return.
#if (EEPROMEND > 256)
		pop		XH
#endif
		pop		XL
		clc
		ret
		.endfunc

		.end
