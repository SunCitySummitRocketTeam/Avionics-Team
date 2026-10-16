#include <Arduino.h>
#include "FlightLogic.h"
#include "Globals.h"
#include "Config.h"
#include "Pins.h"
#include "Kalman.h"
#include "Camera.h"
#include "Buzzer.h"
#include "PretriggerBuffer.h"
#include "Sensors.h"

// ===============================
// FLIGHT LOGIC (SETS FLAGS)
// ===============================
void updateFlightLogic() {
    if (!baselineValid) return;

    // ---------------------------
    // LAUNCH DETECTION
    // ---------------------------
    if (!launched) {
        bool altTrigger = false;
        if (ENABLE_ALT_TRIGGER) {
            altTrigger = (currentSample.relAlt_m >= TRIGGER_ALT_M ||
                          currentSample.relAlt_m <= -TRIGGER_ALT_M);
        }

        bool accelTrigger = false;
        if (ENABLE_ACCEL_TRIGGER) {
            if (currentSample.accelMag >= TRIGGER_ACCEL_G)
                accelTriggerCount++;
            else
                accelTriggerCount = 0;

            accelTrigger = (accelTriggerCount >= TRIGGER_ACCEL_SAMPLES);
        }

        if (altTrigger || accelTrigger) {
            launched = true;
            launchFlagThisRow = 1;
            triggerTimeMs = currentSample.t_ms;

            Serial.println("\n!!! LAUNCH DETECTED !!!");
            if (altTrigger) {
                Serial.print("  Altitude: ");
                Serial.print(currentSample.relAlt_m / FEET_TO_M, 1);
                Serial.println(" ft");
            }
            if (accelTrigger) {
                Serial.print("  Acceleration: ");
                Serial.print(currentSample.accelMag, 2);
                Serial.println(" G");
            }

            digitalWrite(PIN_LED, HIGH);
            if (!CAMERA_ON_STARTUP) triggerCamera(true);
            buzzerOnLaunch();

            configureBmpHighRate();
            configureImuHighRate();

            resetKalman(currentSample.relAlt_m);
            lastFilteredAlt = currentSample.relAlt_m;

            // Write pretrigger buffer
            writePretriggerToFile(currentSample.t_ms);

            mode = MODE_EVENT;
        }
    }

    // ---------------------------
    // APOGEE DETECTION (EVENT MODE)
    // ---------------------------
    if (mode == MODE_EVENT && launched && !apogeeDetected) {
        float filteredAlt = kalmanFilter(currentSample.relAlt_m);
        derivative = filteredAlt - lastFilteredAlt;
        lastFilteredAlt = filteredAlt;

        // Arm apogee
        if (!apogeeArmed && filteredAlt >= APOGEE_ARM_ALT_M) {
            apogeeArmed = true;
            Serial.println("\n*** APOGEE ARMED ***");
            Serial.print("Altitude: ");
            Serial.print(filteredAlt / FEET_TO_M, 0);
            Serial.println(" ft\n");
        }

        // Detect apogee
        if (apogeeArmed) {
            if (derivative < 0) {
                descentSampleCount++;
                if (descentSampleCount >= APOGEE_DESCENT_SAMPLES) {
                    apogeeDetected = true;
                    apogeeFlagThisRow = 1;
                    apogeeTime = currentSample.t_ms;

                    buzzerOn();

                    Serial.println("\n*** APOGEE DETECTED ***");
                    Serial.print("Altitude: ");
                    Serial.print(filteredAlt, 1);
                    Serial.print("m (");
                    Serial.print(filteredAlt / FEET_TO_M, 0);
                    Serial.println("ft)");
                    Serial.print("Descent rate: ");
                    Serial.print(derivative, 2);
                    Serial.println(" m/s");
                    Serial.print("Confirmed after ");
                    Serial.print(descentSampleCount);
                    Serial.println(" samples");
                    Serial.println("BUZZER ON!\n");
                }
            } else {
                descentSampleCount = 0;
            }
        }
    }

    // ---------------------------
    // LANDING DETECTION
    // ---------------------------
    if (apogeeDetected && !landingDetected) {
        bool altNearGround = (abs(currentSample.relAlt_m) < LANDING_ALT_THRESHOLD_M);
        bool accelCalm = (currentSample.accelMag < LANDING_ACCEL_THRESHOLD_G);

        if (altNearGround && accelCalm) {
            landingSampleCount++;
            if (landingSampleCount >= LANDING_CONFIRM_SAMPLES) {
                landingDetected = true;
                Serial.println("\n*** LANDING DETECTED ***");
            }
        } else {
            landingSampleCount = 0;
        }
    }
}