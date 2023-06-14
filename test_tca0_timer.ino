#define F_CPU 16000000

#include <avr/interrupt.h>
#include <avr/io.h>

unsigned int t = 0;

void setup() {
    Serial1.begin(9600);

    // 1 clock = 1/16MHz = 62.5ns
    // after div256 -> 1 clock = 62.5us * 256 = 16us
    // period = 0xff -> 1 interrupt = 16us * 256 = 4096us = 4.096ms

    TCA0_SINGLE_PER = 0xff;
    TCA0_SINGLE_INTCTRL = TCA_SINGLE_OVF_bm;
    TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm;

    sei();
}

void loop() {
    unsigned int T = 3; // [s]
    if (t == T * F_CPU / 65536) { // T * F_CPU / (DIV * (PERIOD + 1))
        Serial1.println("3s");
        digitalWrite(7, HIGH);
        delay(100);
        digitalWrite(7, LOW);
        t = 0;
    }
}

ISR(TCA0_OVF_vect) {
    t += 1;
    TCA0_SINGLE_INTFLAGS = TCA_SINGLE_OVF_bm;
}
