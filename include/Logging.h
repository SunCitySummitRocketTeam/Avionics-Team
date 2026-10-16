#ifndef LOGGING_H
#define LOGGING_H

#include <Arduino.h>

// Initializes SD card and prepares logging system
bool initSDCard();

// Opens a new CSV file segment
void openNewFile();

// Checks if file size exceeded limit and rotates file
void checkFileRotation();

// Writes a single CSV row (armed + event modes)
void writeDataRow();

// Flushes file periodically
void periodicSync();

#endif