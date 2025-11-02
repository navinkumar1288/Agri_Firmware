#include "config.h"
#include "utils.h"
#include "storage.h"
#include "system_config.h"
#include "modem.h"
#include "radio.h"
#include "scheduler.h"
#include "display.h"
#include "rtc_sync.h"
#include "ble_mod.h"

void setup() {
  Serial.begin(115200);
  delay(200);
  initStorage();
  prefs.begin("irrig", false);
  displayInitHeltec();
  loadSystemConfig();
  loadAllSchedulesFromFS();

  // RTC init
  WireRTC.begin(RTC_SDA, RTC_SCL, 100000);
  rtcAvailable = rtc.begin(&WireRTC);
  if (rtcAvailable) {
    Serial.println("RTC detected");
    if (rtc.lostPower()) rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  } else Serial.println("RTC not detected");

  radioInit();
  modemInit();
  modemConfigureAndConnectMQTT();
  // initBLE();
  lastStatusPublish = millis();
  Serial.println("Setup complete");
}

void loop() {
  modemBackgroundRead();
  Radio.IrqProcess();
  handleLoRaIncoming();

  String iq;
  if (dequeueIncoming(iq)) {
    Serial.println("Processing queued incoming: " + iq);
    processIncomingScheduleString(iq);
  }

  runScheduleLoop();
  periodicTasks();
  displayLoop();
  delay(20);
}
