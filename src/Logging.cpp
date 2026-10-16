#include <Arduino.h>
#include <SPI.h>
#include "Logging.h"
#include "Globals.h"
#include "Pins.h"
#include "Config.h"

// ===============================
// SD CARD INITIALIZATION
// ===============================
bool initSDCard() {
    Serial.println("Init SD card...");
    SPI.begin();
    pinMode(PIN_SD_CS, OUTPUT);
    digitalWrite(PIN_SD_CS, HIGH);
    delay(50);

    if (!SD.begin(SdSpiConfig(PIN_SD_CS, SHARED_SPI, SD_SCK_MHZ(4), &SPI))) {
        Serial.println("ERROR: SD card failed!");
        SD.initErrorHalt(&Serial);
        return false;
    }

    Serial.println("SD card OK");
    return true;
}

// ===============================
// OPEN NEW FILE SEGMENT
// ===============================
void openNewFile() {
    if (logFile.isOpen()) {
        logFile.sync();
        logFile.close();
    }

    char filename[32];
    sprintf(filename, "seg%03d.csv", fileSegment);

    if (!logFile.open(filename, O_WRONLY | O_CREAT)) {
        Serial.print("ERROR: Cannot create "); Serial.println(filename);
        sdWriteError = true;
        digitalWrite(PIN_LED, HIGH);
        fileOpen = false;
        return;
    }

    logFile.println("time_ms,alt_m,press_hPa,accelX,accelY,accelZ,accelMag_g,apogee,launch");
    logFile.sync();

    fileOpen = true;
    Serial.print("Opened: "); Serial.println(filename);
}

// ===============================
// FILE ROTATION
// ===============================
void checkFileRotation() {
    if (fileOpen && logFile.fileSize() >= FILE_SIZE_LIMIT) {
        fileSegment++;
        openNewFile();
    }
}

// ===============================
// WRITE CSV ROW
// ===============================
void writeDataRow() {
    if (!fileOpen) return;
    if (mode != MODE_ARMED && mode != MODE_EVENT) return;

    char alt_str[16], press_str[16], ax_str[16], ay_str[16], az_str[16], mag_str[16];
    dtostrf(currentSample.relAlt_m, 1, 4, alt_str);
    dtostrf(currentSample.pressure_hPa, 1, 2, press_str);
    dtostrf(currentSample.accelX, 1, 4, ax_str);
    dtostrf(currentSample.accelY, 1, 4, ay_str);
    dtostrf(currentSample.accelZ, 1, 4, az_str);
    dtostrf(currentSample.accelMag, 1, 3, mag_str);

    char buf[150];
    sprintf(buf, "%lu,%s,%s,%s,%s,%s,%s,%d,%d",
            currentSample.t_ms, alt_str, press_str, ax_str, ay_str, az_str, mag_str,
            apogeeFlagThisRow, launchFlagThisRow);

    size_t written = logFile.println(buf);
    if (written == 0) {
        if (!sdWriteError) {
            Serial.println("!!! SD WRITE FAILED !!!");
            sdWriteError = true;
            digitalWrite(PIN_LED, HIGH);
        }
    }

    // Clear flags
    launchFlagThisRow = 0;
    apogeeFlagThisRow = 0;
}

// ===============================
// PERIODIC SYNC
// ===============================
void periodicSync() {
    uint32_t now = millis() - bootStartMs;

    if (fileOpen && !logFile.sync()) {
        if (!sdWriteError) {
            Serial.println("!!! SD SYNC FAILED !!!");
            sdWriteError = true;
            digitalWrite(PIN_LED, HIGH);
        }
    }
}