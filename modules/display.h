// display.h
#pragma once
#include "heltec.h"
#include "utils.h"

void displayInitHeltec();
void displayLoop();

// display.cpp
#include "display.h"

unsigned long lastDisplayMs = 0;
const unsigned long DISPLAY_REFRESH_MS = 800;

void displayInitHeltec() {
  Heltec.begin(true, false, true);
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0,0,"Irrigation Controller");
  Heltec.display->display();
  delay(400);
  Heltec.display->clear();
  Heltec.display->display();
}

void displayLoop() {
  unsigned long nowMs = millis();
  if (nowMs - lastDisplayMs < DISPLAY_REFRESH_MS) return;
  lastDisplayMs = nowMs;

  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0,0,"Irrigation");
  Heltec.display->drawString(0,12, String("Time:") + formatTimeShort());
  Heltec.display->display();
}
