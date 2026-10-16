#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

// Turns buzzer on at normal frequency
void buzzerOn();

// Turns buzzer on at launch frequency
void buzzerOnLaunch();

// Turns buzzer off
void buzzerOff();

// Runs buzzer test sequence
void buzzerTest();

// Updates buzzer timing (launch + apogee durations)
void updateBuzzer();

#endif