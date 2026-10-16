#ifndef CONFIG_H
#define CONFIG_H

// Camera
const bool CAMERA_ON_STARTUP = true;

// Buzzer
const int BUZZER_FREQ_HZ = 4000;
const int BUZZER_LAUNCH_FREQ_HZ = 2500;
const unsigned long BUZZER_LAUNCH_DURATION_MS = 1000;
const unsigned long BUZZER_APOGEE_DURATION_MS = 5000;

// Unit conversions
const float FEET_TO_M = 0.3048f;
const float G_TO_MS2 = 9.80665f;

// Launch detection
const bool ENABLE_ALT_TRIGGER = false;
const bool ENABLE_ACCEL_TRIGGER = true;
const float TRIGGER_ALT_FEET = 30.0f;
const float TRIGGER_ALT_M = TRIGGER_ALT_FEET * FEET_TO_M;
const float TRIGGER_ACCEL_G = 2.5f;
const int TRIGGER_ACCEL_SAMPLES = 2;

// Timing
const unsigned long WARMUP_TIME_MS = 15000;
const unsigned long CALIBRATION_TIME_MS = 5000;
const unsigned long SETTLING_TIME_MS = 3000;
const unsigned long CALIB_SAMPLE_PERIOD_MS = 20;
const unsigned long LOW_SAMPLE_PERIOD_MS = 10;
const unsigned long HIGH_SAMPLE_PERIOD_MS = 5;
const unsigned long PRETRIGGER_WINDOW_MS = 2000;

// Event limits
const unsigned long EVENT_MIN_TIME_MS = 30000;
const unsigned long FILE_SIZE_LIMIT = 1048576;

// Serial output
const unsigned long SERIAL_STATUS_PERIOD_MS = 500;
const unsigned long SERIAL_ARMED_PERIOD_MS = 1000;

// Apogee detection
const float APOGEE_ARM_ALT_FEET = 200.0f;
const float APOGEE_ARM_ALT_M = APOGEE_ARM_ALT_FEET * FEET_TO_M;
const int APOGEE_DESCENT_SAMPLES = 20;

// Kalman filter
const float KALMAN_Q = 0.5f;
const float KALMAN_R = 1.0f;

// Landing detection
const float LANDING_ALT_THRESHOLD_M = 20.0f;
const float LANDING_ACCEL_THRESHOLD_G = 0.3f;
const int LANDING_CONFIRM_SAMPLES = 400;

// Pretrigger buffer
const int BUFFER_SIZE = (PRETRIGGER_WINDOW_MS / LOW_SAMPLE_PERIOD_MS) + 50;

// Moving average
const int MA_WINDOW = 5;

#endif