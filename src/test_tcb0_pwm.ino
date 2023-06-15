#define F_CPU 16000000

#include <avr/io.h>

void setup() {
    PORTA_DIR |= 0b00000100; // PA2: OUTPUT
    PORTF_DIR |= 0b00010000; // PF4: OUTPUT
    PORTA_OUT &= ~0b00000100; // PA2: LOW
    PORTF_OUT &= ~0b00010000; // PF4: LOW
    TCB0_CCMPH = 0x20; // Duty
    TCB0_CCMPL = 0xff; // TOP
    TCB0_CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm;
    TCB0_CTRLB |= TCB_CCMPEN_bm | TCB_CNTMODE_PWM8_gc;
}

void loop() {
}
