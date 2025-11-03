#include "display.h"
#include "config.h"
#include "utils.h"
#include "HT_SSD1306Wire.h"

// Use Heltec constructor that matches the installed HT_SSD1306Wire.h
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

const unsigned long DISPLAY_REFRESH_MS = 800;
unsigned long lastDisplayMs = 0;

void displayInitHeltec() {
  display.clear();
display.setFont(ArialMT_Plain_10);
  display.drawString(0,0,"Irrigation Controller");
  display.display(); delay(400); display.clear(); 
display.display();
}

void displayLoop() {
  unsigned long nowMs = millis();
  if (nowMs - lastDisplayMs < DISPLAY_REFRESH_MS) return;
  lastDisplayMs = nowMs;
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0,0,"Irrigation");
  display.drawString(0,12, String("Time:") + String(formatTimeShort()) + " S:" + (scheduleRunning?"RUN":"IDLE"));
  display.drawString(0,26, String("SCH:") + (currentScheduleId.length()?currentScheduleId:"NONE"));
  String nodeLine = "Node:N/A";
  if (currentStepIndex >= 0 && currentStepIndex < (int)SeqStep.size())
     nodeLine = "Node:" + String(SeqStep[currentStepIndex].node_id);
  
  display.drawString(0,40, nodeLine);
  display.display();
}
