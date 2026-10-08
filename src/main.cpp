#include <Arduino.h>

void setup() {
    pinMode(PB3, OUTPUT);   // Onboard LED on Nucleo-32 L432KC
}

void loop() {
    digitalWrite(PB3, HIGH);
    delay(100);
    digitalWrite(PB3, LOW);
    delay(100);
}