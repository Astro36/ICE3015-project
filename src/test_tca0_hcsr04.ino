#define F_CPU 16000000

#include <avr/interrupt.h>
#include <avr/io.h>

unsigned int t = 0;

void setup() {
    PORTA_DIR &= ~0b00000001; // PA0: INPUT
    PORTA_DIR |= 0b00000010; // PA1: OUTPUT
    PORTA_PIN0CTRL |= 0b00000001; // PA0: PULLUP
    PORTA_OUT &= ~0b00000010; // PA1: LOW

    TCA0_SINGLE_PER = 0xff;
    TCA0_SINGLE_INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV16_gc | TCA_SINGLE_ENABLE_bm;

    sei();
}

void loop() {
    unsigned int t_start = t;
    PORTA_OUT |= 0b00000010; // PA1: HIGH
    while (t > t_start + 10) {} // 10us 대기
    PORTA_OUT &= ~0b00000010; // PA1: LOW

    t_start = t;
    while (PORTA_IN & 0b00000001 == 0) {} // 초음파가 되돌아 올 때까지 대기
    unsigned int t_diff = t - t_start;

    unsigned int distance = t_diff / 58; // 거리
}

ISR(TCA0_OVF_vect) { // 1us 마다 호출
    t += 1;
    TCA0_SINGLE_INTFLAGS = TCA_SINGLE_OVF_bm;
}
