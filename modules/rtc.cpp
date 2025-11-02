
// rtc_sync.h
#pragma once
#include "config.h"
#include "utils.h"

bool oneShotNtpSyncAndSetRTC();
void checkRtcDriftAndSync();

// rtc_sync.cpp
#include "rtc_sync.h"

bool connectWiFiOnce() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(200);
    Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}
void disconnectWiFiOnce() { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(100); }

bool oneShotNtpSyncAndSetRTC() {
  if (!connectWiFiOnce()) return false;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo;
  unsigned long tstart = millis();
  bool gotTime = false;
  while ((millis()-tstart) < NTP_TIMEOUT_MS) {
    if (getLocalTime(&timeinfo, 2000)) { gotTime = true; break; }
    delay(200);
  }
  if (!gotTime) { disconnectWiFiOnce(); return false; }
  time_t now = time(nullptr);
  DateTime dt((uint32_t)now);
  if (rtcAvailable) rtc.adjust(dt);
  prefs.putULong("last_ntp_sync", (unsigned long)now);
  disconnectWiFiOnce();
  return true;
}

void checkRtcDriftAndSync() {
  static unsigned long lastSyncCheckMillis = 0;
  if (millis() - lastSyncCheckMillis < SYNC_CHECK_INTERVAL_MS) return;
  lastSyncCheckMillis = millis();
  if (!rtcAvailable) { Serial.println("RTC not available; skipping drift check"); return; }
  DateTime rtcTime = rtc.now();
  time_t rtcEpoch = rtcTime.unixtime();
  time_t sysEpoch = time(nullptr);
  if (sysEpoch <= 0) { oneShotNtpSyncAndSetRTC(); return; }
  long diff = (long)sysEpoch - (long)rtcEpoch;
  long absdrift = diff>=0?diff:-diff;
  if ((uint32_t)absdrift > DRIFT_THRESHOLD_S) {
    if (oneShotNtpSyncAndSetRTC()) publishStatusMsg("EVT|NTP_SYNC|OK");
    else publishStatusMsg("ERR|NTP_SYNC_FAIL");
  }
}
