#include "utils.h"

Preferences prefs;
RTC_DS3231 rtc;
TwoWire WireRTC = TwoWire(1);
bool rtcAvailable = false;

// queue
#define INQ_SZ 16
static String incomingQueue[INQ_SZ];
static int inq_head = 0, inq_tail = 0;

// runtime globals used across modules
bool pumpIsOn = false;
String currentScheduleId = "";
time_t scheduleStartEpoch = 0;
uint32_t pumpOnBeforeMs = PUMP_ON_LEAD_DEFAULT_MS;
uint32_t pumpOffAfterMs = PUMP_OFF_DELAY_DEFAULT_MS;
unsigned long lastStatusPublish = 0;
uint32_t DRIFT_THRESHOLD_S = 300;
uint32_t SYNC_CHECK_INTERVAL_MS = 3600UL * 1000UL;

void dbg(const String &s) { Serial.println(s); }
String nowISO8601() {
  struct tm timeinfo; time_t t = time(nullptr); gmtime_r(&t, &timeinfo);
  char buf[32]; strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}
String formatTimeShort() {
  time_t now = time(nullptr); struct tm tmnow; localtime_r(&now, &tmnow);
  char buf[6]; snprintf(buf, sizeof(buf), "%02d:%02d", tmnow.tm_hour, tmnow.tm_min);
  return String(buf);
}

bool enqueueIncoming(const String &s){
  int next = (inq_tail + 1) % INQ_SZ;
  if (next == inq_head) inq_head = (inq_head + 1) % INQ_SZ;
  incomingQueue[inq_tail] = s;
  inq_tail = next;
  return true;
}
bool dequeueIncoming(String &out){
  if (inq_head == inq_tail) return false;
  out = incomingQueue[inq_head];
  inq_head = (inq_head + 1) % INQ_SZ;
  return true;
}
