#ifndef KALMAN_H
#define KALMAN_H

#include "Globals.h"

// Initializes Kalman filter with a starting altitude
void resetKalman(float initialValue);

// Applies Kalman filter to a new altitude measurement
float kalmanFilter(float measurement);

#endif