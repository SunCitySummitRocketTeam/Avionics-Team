#ifndef SAMPLE_H
#define SAMPLE_H

#include <Arduino.h>

struct Sample {
    uint32_t t_ms;
    float relAlt_m;
    float pressure_hPa;
    float accelX, accelY, accelZ;
    float accelMag;
};

#endif