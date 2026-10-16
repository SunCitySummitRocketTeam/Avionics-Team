#include <Arduino.h>
#include "MovingAverage.h"
#include "Config.h"
#include "Globals.h"

// MOVING AVERAGE
float applyMovingAverage(float newVal) {
  altHistory[altHistIdx] = newVal;
  altHistIdx = (altHistIdx + 1) % MA_WINDOW;
  if (altHistIdx == 0) altHistFilled = true;
  float sum = 0;
  int count = altHistFilled ? MA_WINDOW : altHistIdx;
  for (int i = 0; i < count; i++) sum += altHistory[i];
  return sum / count;
}