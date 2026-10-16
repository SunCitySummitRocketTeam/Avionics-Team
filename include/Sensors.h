#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "Sample.h"

// Initializes BMP585 + LSM6DSOX sensors
void initSensors();

// Configures BMP585 for low-power mode
void configureBmpLowPower();

// Configures BMP585 for high-rate mode
void configureBmpHighRate();

// Configures IMU for low-power mode
void configureImuLowPower();

// Configures IMU for high-rate mode
void configureImuHighRate();

// Reads sensors and updates currentSample
bool readSensors();

// Performs baseline calibration accumulation
void accumulateCalibration();

// Finalizes baseline calibration
void finalizeCalibration();

#endif