#include <Arduino.h>
#include "StateMachine.h"
#include "Globals.h"
#include "Pins.h"
#include "Config.h"
#include "Sensors.h"
#include "Logging.h"
#include "FlightLogic.h"
#include "Buzzer.h"
#include "Camera.h"
#include "MovingAverage.h"
#include "PretriggerBuffer.h"
#include "Kalman.h"

// ===============================
// INITIALIZATION
// ===============================
void initStateMachine() {
    mode = MODE_WARMUP;
    modeStartMs = 0;
    bootStartMs = millis();

    Serial.println("\n--- Starting warm-up ---");
}

// ===============================
// RESET FOR NEXT LAUNCH
// ===============================
void resetForNextLaunch() {
    Serial.println("\n=== RESETTING FOR NEXT LAUNCH ===\n");

    buzzerOff();
    if (!CAMERA_ON_STARTUP) triggerCamera(false);

    bufHead = 0;
    bufFilled = false;

    baselineAlt_m = 0.0f;
    baselinePressure_hPa = 0.0f;
    baselineAccelX = 0.0f;
    baselineAccelY = 0.0f;
    baselineAccelZ = 0.0f;
    baselineValid = false;

    calibSumAlt = 0.0f;
    calibSumPressure = 0.0f;
    calibSumAccelX = 0.0f;
    calibSumAccelY = 0.0f;
    calibSumAccelZ = 0.0f;
    calibCount = 0;

    altHistIdx = 0;
    altHistFilled = false;
    for (int i = 0; i < MA_WINDOW; i++) altHistory[i] = 0;

    kalman_x = 0.0f;
    kalman_p = 1.0f;

    accelTriggerCount = 0;
    apogeeArmed = false;
    apogeeDetected = false;
    lastFilteredAlt = 0.0f;
    derivative = 0.0f;
    descentSampleCount = 0;
    apogeeTime = 0;
    landingSampleCount = 0;
    landingDetected = false;

    launchBuzzerActive = false;
    sdWriteError = false;
    fileSegment = 0;
    launched = false;

    if (logFile.isOpen()) {
        logFile.sync();
        logFile.close();
        fileOpen = false;
    }

    bootStartMs = millis();
    triggerTimeMs = 0;
    modeStartMs = 0;
    mode = MODE_WARMUP;

    configureBmpLowPower();
    configureImuLowPower();
}

// ===============================
// STATE MACHINE UPDATE
// ===============================
void updateStateMachine() {
    static uint32_t lastSampleMs = 0;
    static uint32_t lastFlushMs = 0;
    static uint32_t lastStatusMs = 0;
    static uint32_t lastLedToggle = 0;
    static bool ledState = false;

    uint32_t now = millis() - bootStartMs;

    updateBuzzer();

    // ---------------------------
    // MODE: WARMUP
    // ---------------------------
    if (mode == MODE_WARMUP) {
        if (now - lastLedToggle >= 200) {
            lastLedToggle = now;
            ledState = !ledState;
            digitalWrite(PIN_LED, ledState);
        }

        if (now - lastStatusMs >= SERIAL_STATUS_PERIOD_MS) {
            lastStatusMs = now;
            if (readSensors()) {
                Serial.print("Warmup [");
                Serial.print((WARMUP_TIME_MS - now) / 1000);
                Serial.print("s]: Alt=");
                Serial.print(bmp.readAltitude(1013.25f), 1);
                Serial.print("m, Press=");
                Serial.print(currentSample.pressure_hPa, 2);
                Serial.print("hPa, Accel=");
                Serial.print(currentSample.accelMag, 2);
                Serial.println("G");
            }
        }

        if (now >= WARMUP_TIME_MS) {
            digitalWrite(PIN_LED, LOW);
            mode = MODE_CALIBRATE;
            modeStartMs = now;
            calibSumAlt = calibSumPressure = calibSumAccelX = calibSumAccelY = calibSumAccelZ = 0;
            calibCount = 0;
            Serial.println("\n--- Calibrating ---");
        }

        delay(50);
        return;
    }

    // ---------------------------
    // MODE: CALIBRATE
    // ---------------------------
    if (mode == MODE_CALIBRATE) {
        if (now - lastLedToggle >= 150) {
            lastLedToggle = now;
            ledState = !ledState;
            digitalWrite(PIN_LED, ledState);
        }

        if (now - lastSampleMs >= CALIB_SAMPLE_PERIOD_MS) {
            lastSampleMs = now;
            if (readSensors()) {
                accumulateCalibration();
                if (calibCount % 25 == 0) Serial.print(".");
            }
        }

        if (now - modeStartMs >= CALIBRATION_TIME_MS && calibCount > 0) {
            finalizeCalibration();
            resetKalman(0.0f);
            lastFilteredAlt = 0.0f;

            mode = MODE_SETTLING;
            modeStartMs = now;
            Serial.println("\n--- Settling ---");
        }

        delay(5);
        return;
    }

    // ---------------------------
    // MODE: SETTLING
    // ---------------------------
    if (mode == MODE_SETTLING) {
        if (now - lastLedToggle >= 300) {
            lastLedToggle = now;
            ledState = !ledState;
            digitalWrite(PIN_LED, ledState);
        }

        if (now - lastStatusMs >= 500) {
            lastStatusMs = now;
            if (readSensors()) {
                Serial.print("Settling: Alt=");
                Serial.print(currentSample.relAlt_m, 2);
                Serial.print("m, Press=");
                Serial.print(currentSample.pressure_hPa, 2);
                Serial.print("hPa, Accel=");
                Serial.print(currentSample.accelMag, 2);
                Serial.println("G");
            }
        }

        if (now - modeStartMs >= SETTLING_TIME_MS) {
            fileSegment = 0;
            openNewFile();

            mode = MODE_ARMED;
            modeStartMs = now;
            lastFlushMs = now;
            digitalWrite(PIN_LED, LOW);

            Serial.println("\n=========================================");
            Serial.println("  *** ARMED - WAITING FOR LAUNCH ***");
            Serial.print("  Triggers: ");
            if (ENABLE_ALT_TRIGGER) {
                Serial.print(TRIGGER_ALT_FEET);
                Serial.print("ft");
            }
            if (ENABLE_ALT_TRIGGER && ENABLE_ACCEL_TRIGGER) Serial.print(" OR ");
            if (ENABLE_ACCEL_TRIGGER) {
                Serial.print(TRIGGER_ACCEL_G);
                Serial.print("G");
            }
            Serial.println();
            Serial.print("  Apogee arms at: ");
            Serial.print(APOGEE_ARM_ALT_FEET);
            Serial.println(" ft");
            Serial.print("  Apogee detection: ");
            Serial.print(APOGEE_DESCENT_SAMPLES);
            Serial.println(" samples descent");
            Serial.println("  Recording: INDEFINITE (until landing)");
            Serial.println("  File rotation: 1MB per file");
            Serial.println("=========================================\n");
        }

        delay(50);
        return;
    }

    // ---------------------------
    // MODE: ARMED
    // ---------------------------
    if (mode == MODE_ARMED) {
        if (now - lastLedToggle >= 1000) {
            lastLedToggle = now;
            ledState = true;
            digitalWrite(PIN_LED, HIGH);
        }
        if (ledState && now - lastLedToggle >= 50) {
            ledState = false;
            digitalWrite(PIN_LED, LOW);
        }

        if (now - lastSampleMs < LOW_SAMPLE_PERIOD_MS) {
            delay(1);
            return;
        }
        lastSampleMs = now;

        if (!readSensors()) {
            delay(10);
            return;
        }

        float filteredAlt = applyMovingAverage(currentSample.relAlt_m);
        currentSample.relAlt_m = filteredAlt;

        pushSample(currentSample);

        updateFlightLogic();

        writeDataRow();

        checkFileRotation();

        if (now - lastFlushMs >= 500) {
            if (fileOpen) logFile.sync();
            lastFlushMs = now;
        }

        if (now - lastStatusMs >= SERIAL_ARMED_PERIOD_MS) {
            lastStatusMs = now;
            Serial.print("Armed [");
            Serial.print(now / 1000);
            Serial.print("s]: Alt=");
            Serial.print(filteredAlt, 2);
            Serial.print("m (");
            Serial.print(filteredAlt / FEET_TO_M, 1);
            Serial.print("ft), Press=");
            Serial.print(currentSample.pressure_hPa, 2);
            Serial.print("hPa, Accel=");
            Serial.print(currentSample.accelMag, 2);
            Serial.println("G");
        }

        return;
    }

    // ---------------------------
    // MODE: EVENT
    // ---------------------------
    if (mode == MODE_EVENT) {
        uint32_t elapsed = now - triggerTimeMs;

        bool minTimeReached = (elapsed >= EVENT_MIN_TIME_MS);
        bool canStop = minTimeReached && landingDetected;

        if (canStop) {
            if (logFile.isOpen()) {
                logFile.sync();
                logFile.close();
                fileOpen = false;
            }
            digitalWrite(PIN_LED, LOW);
            buzzerOff();
            if (!CAMERA_ON_STARTUP) triggerCamera(false);

            Serial.println("\n*** EVENT COMPLETE ***");
            Serial.print("Duration: ");
            Serial.print(elapsed / 1000.0f, 1);
            Serial.println(" s");
            Serial.print("File segments: ");
            Serial.println(fileSegment + 1);

            if (apogeeDetected) {
                Serial.print("Apogee altitude: ");
                Serial.print(lastFilteredAlt, 1);
                Serial.print(" m (");
                Serial.print(lastFilteredAlt / FEET_TO_M, 0);
                Serial.println(" ft)");
            }

            if (sdWriteError) {
                Serial.println("WARNING: SD write errors occurred during flight!");
            }

            Serial.println("\nRestarting in 10 seconds...");
            for (int i = 10; i > 0; i--) {
                Serial.print(i);
                Serial.println("...");
                delay(1000);
            }

            resetForNextLaunch();
            return;
        }

        if (now - lastSampleMs < HIGH_SAMPLE_PERIOD_MS) {
            delay(1);
            return;
        }
        lastSampleMs = now;

        if (!readSensors()) {
            delay(2);
            return;
        }

        updateFlightLogic();

        writeDataRow();

        checkFileRotation();

        if (now - lastFlushMs >= 500) {
            if (fileOpen && !logFile.sync()) {
                if (!sdWriteError) {
                    Serial.println("!!! SD SYNC FAILED !!!");
                    sdWriteError = true;
                    digitalWrite(PIN_LED, HIGH);
                }
            }
            lastFlushMs = now;
        }

        if (now - lastStatusMs >= SERIAL_STATUS_PERIOD_MS) {
            lastStatusMs = now;
            Serial.print("[T+");
            Serial.print(elapsed / 1000);
            Serial.print("s] Alt=");
            Serial.print(lastFilteredAlt, 1);
            Serial.print("m (");
            Serial.print(lastFilteredAlt / FEET_TO_M, 0);
            Serial.print("ft) dAlt=");
            Serial.print(derivative, 1);
            Serial.print("m/s P=");
            Serial.print(currentSample.pressure_hPa, 1);
            Serial.print("hPa");

            if (apogeeArmed && !apogeeDetected) {
                Serial.print(" [ARMED:");
                Serial.print(descentSampleCount);
                Serial.print("/");
                Serial.print(APOGEE_DESCENT_SAMPLES);
                Serial.print("]");
            }

            if (apogeeDetected) Serial.print(" [APOGEE]");
            if (landingDetected) Serial.print(" [LANDED]");
            if (sdWriteError) Serial.print(" [SD_ERR]");
            Serial.println();
        }

        return;
    }
}