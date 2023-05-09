// F_CPU = 16MHz
#define F_CPU 16000000UL

void setup() {
    Serial1.begin(9600);
}

void loop() {
    digitalWrite(7, HIGH);
    delay(1000);
    digitalWrite(7, LOW);
    delay(1000);

    Serial1.println("123");
}
