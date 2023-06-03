#define F_CPU 16000000

#include <avr/interrupt.h>
#include <avr/io.h>

int t = 0;

void setup() {
    Serial1.begin(9600);

    // 1 clock = 1/16MHz = 62.5ns
    // after div256 -> 1 clock = 62.5us * 256 = 16us
    // period = 0xff -> 1 interrupt = 16us * 256 = 4096us = 4.096ms

    TCA0.SINGLE.PER = 0xff;
    TCA0.SINGLE.INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm;

    sei();
}

void loop() {
    int T = 10; // [s]
    if (t == T * F_CPU / 65536) { // T * F_CPU / (DIV * (PERIOD + 1)) = T * 244
        Serial1.println("10s");
        t = 0;
    }
}

ISR(TCA0_OVF_vect) {
    t += 1;
    TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm;
}
