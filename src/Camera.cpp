#include <Arduino.h>
#include "Camera.h"
#include "Pins.h"

void triggerCamera(bool state) {
    digitalWrite(PIN_CAMERA, state ? HIGH : LOW);
    Serial.println(state ? "CAMERA TRIGGERED ON" : "CAMERA TRIGGERED OFF");
}