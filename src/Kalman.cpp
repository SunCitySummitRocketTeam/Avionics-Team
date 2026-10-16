#include <Arduino.h>
#include "Kalman.h"
#include "Config.h"
#include "Globals.h"

// KALMAN FILTER
float kalmanFilter(float measurement) {
  kalman_p = kalman_p + KALMAN_Q;
  float k = kalman_p / (kalman_p + KALMAN_R);
  kalman_x = kalman_x + k * (measurement - kalman_x);
  kalman_p = (1.0f - k) * kalman_p;
  return kalman_x;
}

void resetKalman(float initialValue) {
  kalman_x = initialValue;
  kalman_p = 1.0f;
}
