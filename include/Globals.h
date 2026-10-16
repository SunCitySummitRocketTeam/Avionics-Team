#ifndef GLOBALS_H
#define GLOBALS_H

#include <SdFat.h>
#include <Adafruit_BMP5xx.h>
#include <Adafruit_LSM6DSOX.h>
#include "Sample.h"
#include "Config.h"

// Sensor objects
extern Adafruit_BMP5xx bmp;
extern Adafruit_LSM6DSOX imu;

// SD card objects
extern SdFs SD;
extern FsFile logFile;

// Buffers
extern Sample ringBuf[BUFFER_SIZE];
extern int bufHead;
extern bool bufFilled;

// Current sample
extern Sample currentSample;

// Baseline calibration
extern float baselineAlt_m;
extern float baselinePressure_hPa;
extern float baselineAccelX;
extern float baselineAccelY;
extern float baselineAccelZ;
extern bool baselineValid;

extern float calibSumAlt;
extern float calibSumPressure;
extern float calibSumAccelX;
extern float calibSumAccelY;
extern float calibSumAccelZ;
extern uint32_t calibCount;

// State machine
enum Mode {
    MODE_WARMUP,
    MODE_CALIBRATE,
    MODE_SETTLING,
    MODE_ARMED,
    MODE_EVENT
};

extern Mode mode;
extern uint32_t bootStartMs;
extern uint32_t modeStartMs;
extern uint32_t triggerTimeMs;

// Flight state
extern bool launched;
extern bool apogeeArmed;
extern bool apogeeDetected;
extern bool landingDetected;
extern int accelTriggerCount;
extern int descentSampleCount;
extern int landingSampleCount;
extern uint32_t apogeeTime;

// Kalman filter
extern float kalman_x;
extern float kalman_p;
extern float lastFilteredAlt;
extern float derivative;

// Moving average
extern float altHistory[MA_WINDOW];
extern int altHistIdx;
extern bool altHistFilled;

// Buzzer
extern bool launchBuzzerActive;
extern uint32_t launchBuzzerStartTime;

// File management
extern int fileSegment;
extern bool fileOpen;
extern bool sdWriteError;

// Flags
extern int launchFlagThisRow;
extern int apogeeFlagThisRow;

#endif