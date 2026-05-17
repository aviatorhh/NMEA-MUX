#define __SFR_OFFSET 0

#include <avr/io.h>

#define BUILTIN_LED PB7
#define RESET_BTN_PIN PL6

.global start
.global stop
.global led_toggle
.global led_on
.global led_off

start:
    ; Button as input
    clr XH
    lds XL, DDRL
    andi XL, ~_BV(RESET_BTN_PIN)
    sts DDRL, XL

    ; Pull-up
    lds XL, PORTL
    ori XL, _BV(RESET_BTN_PIN)
    sts PORTL, XL

    ; LED
    sbi _SFR_IO_ADDR(DDRB), BUILTIN_LED
    ; off
    cbi _SFR_IO_ADDR(PORTB), BUILTIN_LED


#ifdef ETHERNET
    sbi _SFR_IO_ADDR(DDRC), PC0
#endif
    ret

stop:
    ldi EIMSK, 0    

    cbi _SFR_IO_ADDR(PORTB), RESET_BTN_PIN
    cbi _SFR_IO_ADDR(DDRB), BUILTIN_LED

#ifdef ETHERNET
    cbi _SFR_IO_ADDR(DDRC), PC0
#endif
    ret

led_on:
    sbi _SFR_IO_ADDR(PORTB), BUILTIN_LED
    ret

led_off:
    cbi _SFR_IO_ADDR(PORTB), BUILTIN_LED
    ret

led_toggle:
    in XL, PORTB
    ldi XH, _BV(BUILTIN_LED)
    eor XL, XH
    out PORTB, XL
    ret