#include "config.h"
#include "storage.h"
#include "system_config.h"
#include "modem.h"
#include "radio.h"
#include "scheduler.h"
#include "display.h"
#include "rtc_sync.h"
#include "ble_mod.h"
#include "utils.h"

void setup() {
  Serial.begin(115200); delay(200);
  initStorage(); prefs.begin("irrig", false);
  displayInitHeltec();
  loadSystemConfig();
  if (!LittleFS.exists("/schedules")) LittleFS.mkdir("/schedules");
  loadAllSchedulesFromFS();

  // RTC TwoWire init
  WireRTC.begin(RTC_SDA, RTC_SCL, 100000);
  delay(20);
  rtcAvailable = rtc.begin(&WireRTC);
  if (rtcAvailable) {
    Serial.println("RTC detected on WireRTC");
    if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  } else Serial.println("RTC not detected on WireRTC bus");

  radioInit();
  modemInit();
  modemConfigureAndConnectMQTT();
  //initBLE();
  lastStatusPublish = millis();
  Serial.println("Setup complete");
}

void loop() {
  modemBackgroundRead();
  Radio.IrqProcess();
  handleLoRaIncoming();

  // process one incoming queued message
  String iq; if (dequeueIncoming(iq)) { Serial.println("Processing queued incoming: " + iq); processIncomingScheduleString(iq); }

  runScheduleLoop();

  periodicTasks(); // from utils/scheduler (status publish, scheduler check, rtc drift)

  displayLoop();
  delay(20);
}
