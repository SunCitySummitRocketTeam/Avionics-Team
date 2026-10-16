#include <Arduino.h>
#include <Wire.h>
#include "Sensors.h"
#include "Globals.h"
#include "Config.h"

// ===============================
// SENSOR INITIALIZATION
// ===============================
void initSensors() {
    Wire.begin();
    Wire.setClock(400000UL);

    Serial.println("Init BMP585...");
    if (!bmp.begin(BMP5XX_DEFAULT_ADDRESS, &Wire)) {
        if (!bmp.begin(BMP5XX_ALTERNATIVE_ADDRESS, &Wire)) {
            Serial.println("ERROR: BMP585 not found!");
            while (1) { delay(500); }
        }
    }
    configureBmpLowPower();
    Serial.println("BMP585 OK");

    Serial.println("Init LSM6DSOX...");
    if (!imu.begin_I2C()) {
        Serial.println("ERROR: LSM6DSOX not found!");
        while (1) { delay(500); }
    }
    configureImuLowPower();
    Serial.println("LSM6DSOX OK");
}

// ===============================
// BMP585 CONFIG
// ===============================
void configureBmpLowPower() {
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
    bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    Serial.println("BMP585: Low-power mode (50 Hz)");
}

void configureBmpHighRate() {
    bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
    bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_8X);
    bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP5XX_ODR_240_HZ);
    bmp.setPowerMode(BMP5XX_POWERMODE_NORMAL);
    Serial.println("BMP585: High-rate mode (240 Hz)");
    delay(50);
}

// ===============================
// IMU CONFIG
// ===============================
void configureImuLowPower() {
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    imu.setAccelDataRate(LSM6DS_RATE_104_HZ);
    Serial.println("LSM6DSOX: Low-power mode (104 Hz)");
}

void configureImuHighRate() {
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
    imu.setAccelDataRate(LSM6DS_RATE_416_HZ);
    Serial.println("LSM6DSOX: High-rate mode (416 Hz)");
    delay(50);
}

// ===============================
// SENSOR READING
// ===============================
bool readSensors() {
    if (!bmp.performReading()) return false;

    currentSample.pressure_hPa = bmp.pressure / 100.0f;
    float alt_m = bmp.readAltitude(1013.25f);

    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);

    currentSample.t_ms = millis() - bootStartMs;
    currentSample.accelX = accel.acceleration.x;
    currentSample.accelY = accel.acceleration.y;
    currentSample.accelZ = accel.acceleration.z;

    if (baselineValid) {
        currentSample.relAlt_m = alt_m - baselineAlt_m;

        float dX = currentSample.accelX - baselineAccelX;
        float dY = currentSample.accelY - baselineAccelY;
        float dZ = currentSample.accelZ - baselineAccelZ;

        currentSample.accelMag = sqrt(dX*dX + dY*dY + dZ*dZ) / G_TO_MS2;
    } else {
        currentSample.relAlt_m = 0;
        currentSample.accelMag = sqrt(
            currentSample.accelX*currentSample.accelX +
            currentSample.accelY*currentSample.accelY +
            currentSample.accelZ*currentSample.accelZ
        ) / G_TO_MS2;
    }

    return true;
}

// ===============================
// CALIBRATION
// ===============================
void accumulateCalibration() {
    calibSumAlt += bmp.readAltitude(1013.25f);
    calibSumPressure += currentSample.pressure_hPa;
    calibSumAccelX += currentSample.accelX;
    calibSumAccelY += currentSample.accelY;
    calibSumAccelZ += currentSample.accelZ;
    calibCount++;
}

void finalizeCalibration() {
    baselineAlt_m = calibSumAlt / calibCount;
    baselinePressure_hPa = calibSumPressure / calibCount;
    baselineAccelX = calibSumAccelX / calibCount;
    baselineAccelY = calibSumAccelY / calibCount;
    baselineAccelZ = calibSumAccelZ / calibCount;
    baselineValid = true;

    Serial.print("Baseline: Alt=");
    Serial.print(baselineAlt_m, 2);
    Serial.print("m, Press=");
    Serial.print(baselinePressure_hPa, 2);
    Serial.print("hPa, Accel=(");
    Serial.print(baselineAccelX, 2);
    Serial.print(", ");
    Serial.print(baselineAccelY, 2);
    Serial.print(", ");
    Serial.print(baselineAccelZ, 2);
    Serial.println(")");
}