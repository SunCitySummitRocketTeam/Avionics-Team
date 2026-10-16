#ifndef PRETRIGGER_BUFFER_H
#define PRETRIGGER_BUFFER_H

#include "Sample.h"
#include "Globals.h"

// Pushes a sample into the ring buffer
void pushSample(const Sample &s);

// Writes pretrigger samples to file around event time
void writePretriggerToFile(uint32_t eventTimeMs);

#endif