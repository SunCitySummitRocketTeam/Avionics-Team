#include <Arduino.h>

void setup() {
    pinMode(PB3, OUTPUT);   // Onboard LED on Nucleo-32 L432KC
}

void loop() {
    digitalWrite(PB3, HIGH);
    delay(1000);
    digitalWrite(PB3, LOW);
    delay(1000);
}

/*
// put function declarations here:
int myFunction(int, int);

void setup() {
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
*/