#include <Arduino.h>

#include "Pins.h"
#include "Config.h"
#include "Globals.h"

#include "Sensors.h"
#include "Logging.h"
#include "FlightLogic.h"
#include "Buzzer.h"
#include "Camera.h"
#include "StateMachine.h"

// ===============================
// GLOBAL OBJECT DEFINITIONS
// ===============================

// Sensors
Adafruit_BMP5xx bmp;
Adafruit_LSM6DSOX imu;

// SD
SdFs SD;
FsFile logFile;

// Buffers
Sample ringBuf[BUFFER_SIZE];
int bufHead = 0;
bool bufFilled = false;

// Current sample
Sample currentSample;

// Baseline calibration
float baselineAlt_m = 0.0f;
float baselinePressure_hPa = 0.0f;
float baselineAccelX = 0.0f;
float baselineAccelY = 0.0f;
float baselineAccelZ = 0.0f;
bool baselineValid = false;

float calibSumAlt = 0.0f;
float calibSumPressure = 0.0f;
float calibSumAccelX = 0.0f;
float calibSumAccelY = 0.0f;
float calibSumAccelZ = 0.0f;
uint32_t calibCount = 0;

// State machine
Mode mode = MODE_WARMUP;
uint32_t bootStartMs = 0;
uint32_t modeStartMs = 0;
uint32_t triggerTimeMs = 0;

// Flight state
bool launched = false;
bool apogeeArmed = false;
bool apogeeDetected = false;
bool landingDetected = false;
int accelTriggerCount = 0;
int descentSampleCount = 0;
int landingSampleCount = 0;
uint32_t apogeeTime = 0;

// Kalman
float kalman_x = 0.0f;
float kalman_p = 1.0f;
float lastFilteredAlt = 0.0f;
float derivative = 0.0f;

// Moving average
float altHistory[MA_WINDOW] = {0};
int altHistIdx = 0;
bool altHistFilled = false;

// Buzzer
bool launchBuzzerActive = false;
uint32_t launchBuzzerStartTime = 0;

// File management
int fileSegment = 0;
bool fileOpen = false;
bool sdWriteError = false;

// Flags
int launchFlagThisRow = 0;
int apogeeFlagThisRow = 0;


// ===============================
// SETUP
// ===============================
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println();
    Serial.println("===========================================");
    Serial.println("  STM32L4 BMP585 + LSM6DSOX Rocket Logger");
    Serial.println("  MODULAR C++ VERSION (Arduino-based)");
    Serial.println("===========================================");
    Serial.println();

    // Pins
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    pinMode(PIN_CAMERA, OUTPUT);
    digitalWrite(PIN_CAMERA, LOW);

    if (CAMERA_ON_STARTUP)
        triggerCamera(true);

    // LED test
    Serial.println("Testing LED...");
    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_LED, HIGH); delay(100);
        digitalWrite(PIN_LED, LOW); delay(100);
    }

    // Print configuration
    Serial.println("Configuration:");
    Serial.print("  Camera mode: ");
    Serial.println(CAMERA_ON_STARTUP ? "ON AT STARTUP" : "ON AT LAUNCH");

    Serial.print("  Alt trigger: ");
    if (ENABLE_ALT_TRIGGER) {
        Serial.print("+/- ");
        Serial.print(TRIGGER_ALT_FEET);
        Serial.println(" ft");
    } else {
        Serial.println("DISABLED");
    }

    Serial.print("  Accel trigger: ");
    if (ENABLE_ACCEL_TRIGGER) {
        Serial.print(TRIGGER_ACCEL_G);
        Serial.print(" G (");
        Serial.print(TRIGGER_ACCEL_SAMPLES);
        Serial.println(" samples)");
    } else {
        Serial.println("DISABLED");
    }

    Serial.print("  Apogee arm altitude: ");
    Serial.print(APOGEE_ARM_ALT_FEET);
    Serial.println(" ft");

    Serial.print("  Apogee descent samples: ");
    Serial.print(APOGEE_DESCENT_SAMPLES);
    Serial.print(" (");
    Serial.print(APOGEE_DESCENT_SAMPLES * HIGH_SAMPLE_PERIOD_MS);
    Serial.println(" ms @ 200Hz)");

    Serial.print("  Pretrigger buffer: ");
    Serial.print(PRETRIGGER_WINDOW_MS / 1000);
    Serial.println(" seconds");

    Serial.print("  File rotation: ");
    Serial.print(FILE_SIZE_LIMIT / 1048576);
    Serial.println(" MB per file");

    Serial.println("  Recording: INDEFINITE (until landing)");
    Serial.println();

    if (!ENABLE_ALT_TRIGGER && !ENABLE_ACCEL_TRIGGER) {
        Serial.println("ERROR: No triggers enabled!");
        while (1) { delay(500); }
    }

    // Start timing
    bootStartMs = millis();

    // Init sensors
    initSensors();

    // Init SD card
    initSDCard();

    // Buzzer test
    buzzerTest();

    // Init state machine
    initStateMachine();
}


// ===============================
// LOOP
// ===============================
void loop() {
    updateStateMachine();
}