#include "display.h"
#include "utils.h"

const unsigned long DISPLAY_REFRESH_MS = 800;
unsigned long lastDisplayMs = 0;

void displayInitHeltec() {
  Heltec.begin(true, false, true);
  Heltec.display->clear(); Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0,0,"Irrigation Controller");
  Heltec.display->display(); delay(400); Heltec.display->clear(); Heltec.display->display();
}

void displayLoop() {
  unsigned long nowMs = millis();
  if (nowMs - lastDisplayMs < DISPLAY_REFRESH_MS) return;
  lastDisplayMs = nowMs;
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0,0,"Irrigation");
  Heltec.display->drawString(0,12, String("Time:") + String(formatTimeShort()) + " S:" + (scheduleRunning?"RUN":"IDLE"));
  Heltec.display->drawString(0,26, String("SCH:") + (currentScheduleId.length()?currentScheduleId:"NONE"));
  String nodeLine = "Node:N/A";
  if (currentStepIndex>=0 && currentStepIndex < (int)seq.size()) nodeLine = "Node:" + String(seq[currentStepIndex].node_id);
  Heltec.display->drawString(0,40, nodeLine);
  Heltec.display->display();
}
