#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <Arduino.h>

// Initializes state machine and hardware
void initStateMachine();

// Runs the state machine (called from loop)
void updateStateMachine();

// Resets everything for next launch
void resetForNextLaunch();

#endif