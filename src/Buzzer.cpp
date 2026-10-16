#include <Arduino.h>
#include "Buzzer.h"
#include "Globals.h"
#include "Pins.h"
#include "Config.h"

// BUZZER FUNCTIONS
void buzzerOn() {
    tone(PIN_BUZZER, BUZZER_FREQ_HZ);
}

void buzzerOnLaunch() {
    tone(PIN_BUZZER, BUZZER_LAUNCH_FREQ_HZ);
    launchBuzzerActive = true;
    launchBuzzerStartTime = millis();
}

void buzzerOff() {
    noTone(PIN_BUZZER);
    launchBuzzerActive = false;
}

void buzzerTest() {
    Serial.println("Testing buzzer...");
    int freqs[] = {247, 370, 494, 440, 370};
    for (int i = 0; i < 5; i++) {
        Serial.print("  "); Serial.print(freqs[i]); Serial.println(" Hz");
        tone(PIN_BUZZER, freqs[i]);
        delay(200);
        noTone(PIN_BUZZER);
        delay(100);
    }
    Serial.print("  Final test at "); Serial.print(BUZZER_FREQ_HZ); Serial.println(" Hz");
    tone(PIN_BUZZER, BUZZER_FREQ_HZ);
    delay(500);
    noTone(PIN_BUZZER);
    Serial.println("Buzzer test complete\n");
}

// BUZZER MANAGEMENT
void updateBuzzer() {
    uint32_t now = millis() - bootStartMs;

    // Launch buzzer auto-off
    if (launchBuzzerActive && !apogeeDetected) {
        if (millis() - launchBuzzerStartTime >= BUZZER_LAUNCH_DURATION_MS) {
            buzzerOff();
        }
    }

    // Apogee buzzer auto-off
    if (apogeeDetected && (now - apogeeTime >= BUZZER_APOGEE_DURATION_MS)) {
        buzzerOff();
    }
}