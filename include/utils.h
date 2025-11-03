#pragma once

extern Preferences prefs;
extern RTC_DS3231 rtc;
extern TwoWire WireRTC;
extern bool rtcAvailable;

void dbg(const String &s);
String nowISO8601();
String formatTimeShort();

// incoming queue helpers
bool enqueueIncoming(const String &s);
bool dequeueIncoming(String &out);
