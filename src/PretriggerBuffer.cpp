#include <Arduino.h>
#include "PretriggerBuffer.h"
#include "Globals.h"
#include "Config.h"

void pushSample(const Sample &s) {
  ringBuf[bufHead] = s;
  bufHead = (bufHead + 1) % BUFFER_SIZE;
  if (bufHead == 0) bufFilled = true;
}

void writePretriggerToFile(uint32_t eventTimeMs) {
  if (!fileOpen) return;
  
  uint32_t minTime = (eventTimeMs > PRETRIGGER_WINDOW_MS) ? (eventTimeMs - PRETRIGGER_WINDOW_MS) : 0;
  int count = bufFilled ? BUFFER_SIZE : bufHead;
  int startIndex = bufFilled ? bufHead : 0;
  
  Serial.println("Writing pretrigger data...");
  int written = 0;
  
  for (int n = 0; n < count; n++) {
    int idx = (startIndex + n) % BUFFER_SIZE;
    Sample &s = ringBuf[idx];
    if (s.t_ms >= minTime && s.t_ms <= eventTimeMs) {
      char alt_str[16], press_str[16], ax_str[16], ay_str[16], az_str[16], mag_str[16];
      dtostrf(s.relAlt_m, 1, 4, alt_str);
      dtostrf(s.pressure_hPa, 1, 2, press_str);
      dtostrf(s.accelX, 1, 4, ax_str);
      dtostrf(s.accelY, 1, 4, ay_str);
      dtostrf(s.accelZ, 1, 4, az_str);
      dtostrf(s.accelMag, 1, 3, mag_str);
      
      char buf[150];
      sprintf(buf, "%lu,%s,%s,%s,%s,%s,%s,0,0",
              s.t_ms, alt_str, press_str, ax_str, ay_str, az_str, mag_str);
      logFile.println(buf);
      written++;
    }
  }
  
  Serial.print("Wrote "); Serial.print(written); Serial.println(" pretrigger samples");
}