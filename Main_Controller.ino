/* 
  Main Controller - Final (Heltec LoRa)
  - Uses Heltec's heltec.h for LoRa (no explicit <LoRa.h>)
  - Modem UART pins: MODEM_RX=45, MODEM_TX=46, MODEM_BAUD=115200
  - Secondary I2C TwoWire (WireRTC) SDA=41 SCL=42 for DS3231 RTC
  - Multi-admin + recovery token, queueing, LoRa MID/ACK, MQTT via AT, BLE, etc.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <RTClib.h>
#include "heltec.h"        // Heltec board + LoRa support via Heltec 
#include "HT_SSD1306Wire.h"          // Heltec SSD1306 driver
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// ---------------- CONFIG / pins ----------------
// Modem (TracX EC200U) UART pins (Heltec defaults you provided)
#define MODEM_RX 45
#define MODEM_TX 46
#define MODEM_BAUD 115200

// I2C for OLED uses default Wire; use a secondary TwoWire for RTC
#define RTC_SDA 41
#define RTC_SCL 42
TwoWire WireRTC = TwoWire(1);
bool rtcAvailable = false;

// ---------------- Display (Heltec) ----------------
// Use Heltec constructor that matches the installed HT_SSD1306Wire.h
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// -------------------- CONFIG --------------------
#define WIFI_SSID "sekarfarm"
#define WIFI_PASS "welcome123"

#define DEFAULT_MQTT_SERVER "39aff691b9b5421ab98adc2addedbd83.s1.eu.hivemq.cloud"
#define DEFAULT_MQTT_PORT 8883
#define DEFAULT_MQTT_USER "navin"
#define DEFAULT_MQTT_PASS "HaiNavin33"
#define DEFAULT_SIM_APN "airtelgprs.com"

#define MQTT_TOPIC_SCHEDULE "irrigation/site01/schedule/set"
#define MQTT_TOPIC_CONFIG   "irrigation/site01/config/system/set"
#define MQTT_TOPIC_STATUS   "irrigation/site01/status"

// LoRa SPI pins
#define LORA_CS   18
#define LORA_RST  14
#define LORA_DIO0 26
// LoRa freq as long (Hz). Use PABOOST true for higher power if your board/antenna supports it.
#define LORA_FREQ 865000000L  // 865 MHz

// Controller pump pin
#define PUMP_PIN 25
#define PUMP_ACTIVE_HIGH true

// BLE
#define BLE_DEVICE_NAME "IrrigCtrl"
#define BLE_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Behavior tuning
const uint32_t LORA_ACK_TIMEOUT_MS = 3000;
const uint8_t  LORA_MAX_RETRIES = 3;
const uint32_t SAVE_PROGRESS_INTERVAL_MS = 10 * 1000;
const uint32_t PUMP_ON_LEAD_DEFAULT_MS = 2000;
const uint32_t PUMP_OFF_DELAY_DEFAULT_MS = 5000;
const uint32_t LAST_CLOSE_DELAY_MS_DEFAULT = 60000;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800; // IST +05:30
const int   daylightOffset_sec = 0;
const int   WIFI_CONNECT_TIMEOUT_MS = 20000;
const int   NTP_TIMEOUT_MS = 20000;

// Recovery token (default; can be changed via config)
#define DEFAULT_RECOV_TOK "FARMTOK123"

// -------------------- GLOBALS --------------------
Preferences prefs;
RTC_DS3231 rtc;
HardwareSerial ModemSerial(2);

String modemLineBuffer = "";
unsigned long lastModemActivity = 0;
unsigned long lastMqttURCTime = 0;
bool mqttAvailable = true;
bool ENABLE_SMS_BROADCAST = false; // set false as SMS fallback not required
uint32_t LAST_CLOSE_DELAY_MS = LAST_CLOSE_DELAY_MS_DEFAULT;
uint32_t DRIFT_THRESHOLD_S = 300;
uint32_t SYNC_CHECK_INTERVAL_MS = 3600UL * 1000UL;

// system config persisted
struct SystemConfig {
  String mqttServer;
  int mqttPort;
  String mqttUser;
  String mqttPass;
  String adminPhones; // comma-separated list of admin phones, stored normalized (+...)
  String simApn;
  String sharedTok; // single shared token
  String recoveryTok; // recovery token to allow admin reset
} sysConfig;

// -------------------- Schedule structures --------------------
struct SeqStep { int node_id; uint32_t duration_ms; };
struct Schedule {
  String id;
  char rec; // 'O' = onetime, 'D' = daily, 'W' = weekly
  time_t start_epoch;
  String timeStr;
  uint8_t weekday_mask;
  std::vector<SeqStep> seq;
  uint32_t pump_on_before_ms;
  uint32_t pump_off_after_ms;
  bool enabled;
  time_t next_run_epoch;
  uint32_t ts; // timestamp/version
};

// runtime collections
std::vector<Schedule> schedules;
String currentScheduleId = "";
time_t scheduleStartEpoch = 0;
uint32_t pumpOnBeforeMs = PUMP_ON_LEAD_DEFAULT_MS;
uint32_t pumpOffAfterMs = PUMP_OFF_DELAY_DEFAULT_MS;

std::vector<SeqStep> seq;          // sequence loaded from active schedule
int currentStepIndex = -1;         // -1 = not started
unsigned long stepStartMillis = 0;
bool scheduleLoaded = false;
bool scheduleRunning = false;

unsigned long lastProgressSave = 0;
unsigned long lastStatusPublish = 0;
unsigned long statusPublishInterval = 15 * 1000; // 15s

// Display
const unsigned long DISPLAY_REFRESH_MS = 800;
static String disp_time_line = "";
static String disp_status_line = "";
static String disp_node_line = "";
static String disp_error = "";
static unsigned long lastDisplayMs = 0;

// ---------- Incoming queue (ring buffer) ----------
#define INQ_SZ 16
String incomingQueue[INQ_SZ];
int inq_head = 0, inq_tail = 0;

bool enqueueIncoming(const String &s){
  int next = (inq_tail + 1) % INQ_SZ;
  if (next == inq_head) { // queue full -> drop oldest
    inq_head = (inq_head + 1) % INQ_SZ;
  }
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

// ---------- Utilities ----------
String nowISO8601() {
  struct tm timeinfo; time_t t = time(nullptr); gmtime_r(&t, &timeinfo);
  char buf[32]; strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
  return String(buf);
}
void debugPrint(const String &s){ Serial.println(s); }

// ---------- Display helpers ----------
void displayInitHeltec() {
  //Heltec.begin(true, false, true);
  display.clear(); display.setFont(ArialMT_Plain_10);
  display.drawString(0,0,"Irrigation Controller");
  display.display(); delay(400); display.clear(); display.display();
}
String formatTimeShort() {
  time_t now = time(nullptr); struct tm tmnow; localtime_r(&now, &tmnow);
  char buf[6]; snprintf(buf, sizeof(buf), "%02d:%02d", tmnow.tm_hour, tmnow.tm_min);
  return String(buf);
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
  if (currentStepIndex>=0 && currentStepIndex < (int)seq.size()) nodeLine = "Node:" + String(seq[currentStepIndex].node_id);
  display.drawString(0,40, nodeLine);
  display.display();
}

// ---------- Storage helpers (LittleFS + Preferences) ----------
bool initStorage() {
  if (!LittleFS.begin(true)) { Serial.println("LittleFS mount failed"); return false; }
  prefs.begin("irrig", false);
  return true;
}
bool saveStringFile(const String &path, const String &content) {
  File f = LittleFS.open(path, "w");
  if (!f) return false;
  f.print(content); f.close(); return true;
}
String loadStringFile(const String &path) {
  if (!LittleFS.exists(path)) return String("");
  File f = LittleFS.open(path, "r"); if (!f) return String("");
  String c = f.readString(); f.close(); return c;
}

// Persist per-schedule JSON under /schedules/<ID>.json
bool saveScheduleFile(const Schedule &s) {
  DynamicJsonDocument d(4096);
  d["schedule_id"] = s.id; d["recurrence"] = (s.rec=='D'?"daily":(s.rec=='W'?"weekly":"onetime"));
  d["start_time"] = s.timeStr; d["start_epoch"] = (long long)s.start_epoch;
  d["pump_on_before_ms"] = s.pump_on_before_ms; d["pump_off_after_ms"] = s.pump_off_after_ms;
  d["ts"] = s.ts;
  JsonArray arr = d.createNestedArray("sequence");
  for (auto &st : s.seq) { JsonObject so = arr.createNestedObject(); so["node_id"]=st.node_id; so["duration_ms"]=st.duration_ms; }
  String out; serializeJson(d, out);
  String path = String("/schedules/") + s.id + String(".json");
  return saveStringFile(path, out);
}
bool deleteScheduleFile(const String &id) {
  String path = String("/schedules/") + id + String(".json");
  if (LittleFS.exists(path)) return LittleFS.remove(path);
  return true;
}
Schedule scheduleFromJsonString(const String &json) {
  Schedule s; s.seq.clear(); s.id=""; s.rec='O'; s.start_epoch=0; s.timeStr=""; s.weekday_mask=0;
  s.pump_on_before_ms=PUMP_ON_LEAD_DEFAULT_MS; s.pump_off_after_ms=PUMP_OFF_DELAY_DEFAULT_MS; s.enabled=true; s.next_run_epoch=0; s.ts = 0;
  StaticJsonDocument<4096> doc; DeserializationError err = deserializeJson(doc, json);
  if (err) return s;
  s.id = String((const char*)(doc["schedule_id"]|doc["id"]|""));
  String recurrence = String((const char*)(doc["recurrence"]|doc["rec"]|""));
  if (recurrence.startsWith("d")||recurrence.startsWith("D")) s.rec='D'; else if (recurrence.startsWith("w")||recurrence.startsWith("W")) s.rec='W'; else s.rec='O';
  s.timeStr = String((const char*)(doc["start_time"]|doc["time"]|""));
  s.start_epoch = (time_t)(doc["start_epoch"].as<long long>()? doc["start_epoch"].as<long long>() : 0);
  s.pump_on_before_ms = doc["pump_on_before_ms"] | PUMP_ON_LEAD_DEFAULT_MS;
  s.pump_off_after_ms = doc["pump_off_after_ms"] | PUMP_OFF_DELAY_DEFAULT_MS;
  s.ts = doc["ts"] | 0;
  if (doc.containsKey("sequence") && doc["sequence"].is<JsonArray>()) {
    for (JsonVariant v : doc["sequence"].as<JsonArray>()) {
      SeqStep st; st.node_id = v["node_id"].as<int>(); st.duration_ms = v["duration_ms"].as<uint32_t>(); s.seq.push_back(st);
    }
  }
  return s;
}
void loadAllSchedulesFromFS() {
  schedules.clear();
  if (!LittleFS.exists("/schedules")) {
    LittleFS.mkdir("/schedules");
    return;
  }
  File root = LittleFS.open("/schedules");
  File file = root.openNextFile();
  while (file) {
    String name = file.name(); if (name.endsWith(".json")) {
      String content = file.readString(); Schedule s = scheduleFromJsonString(content);
      if (s.id.length()) schedules.push_back(s);
    }
    file = root.openNextFile();
  }
}

// -------------------- System config storage --------------------
String normalizePhone(const String &in) {
  String s = in; s.trim(); s.replace(" ", "");
  if (s.length() > 0 && s.charAt(0) == '0') s = s.substring(1);
  String tmp = s; int digits = 0; for (int i=0;i<(int)tmp.length();++i) if (isDigit(tmp[i])) digits++;
  if (s.length() == 10 && s.charAt(0) != '+') s = String("+91") + s;
  return s;
}
std::vector<String> adminPhoneList() {
  std::vector<String> out;
  String s = sysConfig.adminPhones; s.trim();
  if (s.length() == 0) return out;
  int p = 0;
  while (p < (int)s.length()) {
    int c = s.indexOf(',', p);
    String part = (c==-1) ? s.substring(p) : s.substring(p, c);
    part.trim();
    if (part.length()) out.push_back(part);
    if (c == -1) break;
    p = c + 1;
  }
  return out;
}
bool isAdminNumber(const String &num) {
  String n = normalizePhone(num);
  auto list = adminPhoneList();
  for (auto &p : list) if (normalizePhone(p) == n) return true;
  return false;
}

void loadSystemConfig() {
  sysConfig.mqttServer = prefs.getString("mqtt_server", DEFAULT_MQTT_SERVER);
  sysConfig.mqttPort = prefs.getInt("mqtt_port", DEFAULT_MQTT_PORT);
  sysConfig.mqttUser = prefs.getString("mqtt_user", DEFAULT_MQTT_USER);
  sysConfig.mqttPass = prefs.getString("mqtt_pass", DEFAULT_MQTT_PASS);
  sysConfig.adminPhones = prefs.getString("admin_phones", "+919944272647");
  sysConfig.simApn = prefs.getString("sim_apn", DEFAULT_SIM_APN);
  sysConfig.sharedTok = prefs.getString("shared_tok", "MYTOK");
  sysConfig.recoveryTok = prefs.getString("recovery_tok", DEFAULT_RECOV_TOK);
  LAST_CLOSE_DELAY_MS = prefs.getULong("last_close_delay_ms", LAST_CLOSE_DELAY_MS_DEFAULT);
  DRIFT_THRESHOLD_S = prefs.getUInt("drift_s", DRIFT_THRESHOLD_S);
  uint32_t sync_h = prefs.getUInt("sync_h", (uint32_t)(SYNC_CHECK_INTERVAL_MS/3600000UL));
  SYNC_CHECK_INTERVAL_MS = (uint32_t)sync_h * 3600UL * 1000UL;
  Serial.println("Loaded system config.");
}
void saveSystemConfig() {
  prefs.putString("mqtt_server", sysConfig.mqttServer);
  prefs.putInt("mqtt_port", sysConfig.mqttPort);
  prefs.putString("mqtt_user", sysConfig.mqttUser);
  prefs.putString("mqtt_pass", sysConfig.mqttPass);
  prefs.putString("admin_phones", sysConfig.adminPhones);
  prefs.putString("sim_apn", sysConfig.simApn);
  prefs.putString("shared_tok", sysConfig.sharedTok);
  prefs.putString("recovery_tok", sysConfig.recoveryTok);
  prefs.putULong("last_close_delay_ms", LAST_CLOSE_DELAY_MS);
  prefs.putUInt("drift_s", DRIFT_THRESHOLD_S);
  prefs.putUInt("sync_h", (uint32_t)(SYNC_CHECK_INTERVAL_MS/3600000UL));
  Serial.println("Saved system config to prefs.");
}

// -------------------- MODEM helpers --------------------
String sendAT(const String &cmd, unsigned long timeoutMs = 2000) {
  while (ModemSerial.available()) ModemSerial.read();
  ModemSerial.print(cmd + String("\r\n"));
  String out; unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    while (ModemSerial.available()) { char c = (char)ModemSerial.read(); out += c; lastModemActivity = millis(); }
    delay(5);
  }
  return out;
}
void modemInit(){ ModemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX); delay(200); while (ModemSerial.available()) ModemSerial.read(); Serial.println("Modem serial init"); }
bool modemConfigureAndConnectMQTT() {
  sendAT("AT",2000);
  String setPdp = String("AT+QICSGP=1,1,\"") + sysConfig.simApn + String("\",\"\",\"\",1");
  sendAT(setPdp,4000);
  sendAT("AT+QIACT=1",10000);
  String openCmd = String("AT+QMTOPEN=0,\"") + sysConfig.mqttServer + String("\",") + String(sysConfig.mqttPort);
  sendAT(openCmd,10000);
  String connCmd = String("AT+QMTCONN=0,\"irrig_main\",\"") + sysConfig.mqttUser + String("\",\"") + sysConfig.mqttPass + String("\"");
  sendAT(connCmd, 10000);
  sendAT(String("AT+QMTSUB=0,1,\"") + MQTT_TOPIC_SCHEDULE + String("\",1"), 5000);
  sendAT(String("AT+QMTSUB=0,1,\"") + MQTT_TOPIC_CONFIG + String("\",1"), 5000);
  Serial.println("Modem configured for MQTT (verify URCs).");
  return true;
}
bool modemPublish(const char* topic, const String &payload) {
  String cmd = String("AT+QMTPUB=0,0,0,1,\"") + topic + String("\",\"");
  String p = payload; p.replace("\"","\\\"");
  cmd += p + String("\"");
  String resp = sendAT(cmd, 6000);
  return resp.indexOf("OK") >= 0;
}

// ---------- LoRa helpers ----------
void loraInit() {
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  // PABOOST true if using higher PA; change to false if your board doesn't support PA_BOOST.
  if (!LoRa.begin(LORA_FREQ, true)) Serial.println("LoRa init failed");
  else Serial.println("LoRa init OK");
}
void sendLoRaCmdRaw(const String &cmd) { LoRa.beginPacket(); LoRa.print(cmd); LoRa.endPacket(); Serial.printf("LoRa SENT: %s\n", cmd.c_str()); }
uint32_t getNextMsgId() { uint32_t mid = prefs.getUInt("msg_counter", 0); mid++; prefs.putUInt("msg_counter", mid); return mid; }

// parse ACKs
bool parseAckWithMid(const String &msg, uint32_t wantMid, const String &wantType, int wantNode, const String &wantSched, int wantSeqIndex) {
  // expected: ACK|MID=123|OPEN|N=2,I=1,S=SC001|OK
  if (!msg.startsWith("ACK|")) return false;
  std::vector<String> parts;
  int start = 0;
  for (int i=0;i<(int)msg.length();++i) if (msg[i]=='|') { parts.push_back(msg.substring(start,i)); start = i+1; }
  parts.push_back(msg.substring(start));
  if (parts.size() < 4) return false;
  String midPart = parts[1];
  if (!midPart.startsWith("MID=")) return false;
  uint32_t mid = (uint32_t)midPart.substring(4).toInt();
  if (mid != wantMid) return false;
  String type = parts[2];
  if (type != wantType) return false;
  String kv = parts[3];
  int node=-1, idx=-1; String sched="";
  int pos=0;
  while (pos < (int)kv.length()) {
    int c = kv.indexOf(',', pos);
    String t = (c==-1) ? kv.substring(pos) : kv.substring(pos, c);
    t.trim();
    if (t.startsWith("N=")) node = t.substring(2).toInt();
    else if (t.startsWith("I=")) idx = t.substring(2).toInt();
    else if (t.startsWith("S=")) sched = t.substring(2);
    if (c==-1) break; pos = c+1;
  }
  String last = parts.back();
  if (node != wantNode) return false;
  if (idx != wantSeqIndex) return false;
  if (sched != wantSched) return false;
  if (last.indexOf("OK") < 0) return false;
  return true;
}

bool waitForAckWithMid(int wantNode, const String &wantType, const String &wantSched, int wantSeqIndex, uint32_t wantMid, uint32_t timeout_ms) {
  unsigned long start = millis();
  while (millis() - start < timeout_ms) {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      // Read into fixed buffer (avoid repeated String concatenation)
      char msgBuf[512]; int idx = 0;
      while (LoRa.available() && idx < (int)sizeof(msgBuf)-1) {
        int c = LoRa.read();
        if (c < 0) break;
        msgBuf[idx++] = (char)c;
      }
      msgBuf[idx] = 0;
      String msg = String(msgBuf);
      Serial.printf("LoRa RCV: %s\n", msg.c_str());
      if (parseAckWithMid(msg, wantMid, wantType, wantNode, wantSched, wantSeqIndex)) return true;
    }
    delay(10);
  }
  return false;
}

bool sendCmdWithAck(const String &cmdType, int node, const String &schedId, int seqIndex, uint32_t durationMs = 0) {
  uint32_t mid = getNextMsgId();
  String kv = String("N=") + String(node) + String(",S=") + schedId + String(",I=") + String(seqIndex);
  if (cmdType == "OPEN" && durationMs > 0) kv += String(",T=") + String(durationMs);
  String cmd = String("CMD|MID=") + String(mid) + String("|") + cmdType + String("|") + kv;
  Serial.printf("Sending LoRa cmd: %s\n", cmd.c_str());
  uint8_t attempt = 0;
  while (attempt < LORA_MAX_RETRIES) {
    sendLoRaCmdRaw(cmd);
    if (waitForAckWithMid(node, cmdType, schedId, seqIndex, mid, LORA_ACK_TIMEOUT_MS)) return true;
    attempt++; Serial.printf("No ACK (MID=%u) for %s node %d attempt %d\n", (unsigned)mid, cmdType.c_str(), node, attempt);
  }
  return false;
}

// ---------- Incoming handlers (queue) ----------
void processIncomingScheduleString(const String &payload); // forward
void handleLoRaIncoming() {
  int packetSize = LoRa.parsePacket(); if (packetSize==0) return;
  // Read into fixed buffer to reduce heap churn
  char payloadBuf[512]; int idx = 0;
  while (LoRa.available() && idx < (int)sizeof(payloadBuf)-1) {
    int c = LoRa.read(); if (c < 0) break; payloadBuf[idx++] = (char)c;
  }
  payloadBuf[idx] = 0;
  String payload = String(payloadBuf);
  payload.trim(); if (payload.length()==0) return;
  if (payload.indexOf("SRC=") < 0) payload += String(",SRC=LORA");
  enqueueIncoming(payload);
  publishStatusMsg(String("EVT|INQ|ENQ|SRC=LORA"));
}

// BLE callbacks
class ControllerBLECallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    auto v = pChar->getValue();
    String payload = String(v.c_str());
    payload.trim();
    if (payload.length() == 0) return;
    if (payload.indexOf("SRC=") < 0) payload += String(",SRC=BT");
    enqueueIncoming(payload);
    publishStatusMsg(String("EVT|INQ|ENQ|SRC=BT"));
  }
};

// ---------- Modem SMS read helper ----------
String modemReadSMSByIndex(int index, String &outSender) {
  outSender = "";
  String cmd = String("AT+CMGR=") + String(index);
  String resp = sendAT(cmd, 3000);
  int headerPos = resp.indexOf("+CMGR:");
  if (headerPos >= 0) {
    int nl = resp.indexOf('\n', headerPos);
    if (nl > headerPos) {
      String header = resp.substring(headerPos, nl);
      int q1 = header.indexOf('\"');
      if (q1 >= 0) {
        int q2 = header.indexOf('\"', q1 + 1);
        int q3 = header.indexOf('\"', q2 + 1);
        int q4 = header.indexOf('\"', q3 + 1);
        if (q3 >= 0 && q4 > q3) outSender = header.substring(q3 + 1, q4);
      }
    }
  }
  int firstNl = resp.indexOf('\n');
  if (firstNl < 0) return String("");
  String after = resp.substring(firstNl + 1);
  int okpos = after.indexOf("\r\nOK");
  String body = (okpos >= 0) ? after.substring(0, okpos) : after;
  body.trim();
  // delete message
  sendAT(String("AT+CMGD=") + String(index), 2000);
  Serial.printf("SMS from %s body: %s\n", outSender.c_str(), body.c_str());
  return body;
}

// Modem background read: handles +QMTRECV and +CMTI
void modemBackgroundRead() {
  while (ModemSerial.available()) { char c = (char)ModemSerial.read(); modemLineBuffer += c; lastModemActivity = millis(); }
  int nl;
  while ((nl = modemLineBuffer.indexOf('\n')) >= 0) {
    String line = modemLineBuffer.substring(0, nl+1); modemLineBuffer = modemLineBuffer.substring(nl+1);
    line.trim(); if (line.length()==0) continue;
    Serial.println(String("[MODEM] ") + line);
    if (line.startsWith("+QMTRECV:")) {
      lastMqttURCTime = millis();
      int firstQuote = line.indexOf('\"');
      if (firstQuote>=0) {
        int secondQuote = line.indexOf('\"', firstQuote+1);
        if (secondQuote>firstQuote) {
          int thirdQuote = line.indexOf('\"', secondQuote+1);
          int fourthQuote = line.indexOf('\"', thirdQuote+1);
          if (thirdQuote>=0 && fourthQuote>thirdQuote) {
            String payload = line.substring(thirdQuote+1, fourthQuote);
            if (payload.indexOf("SRC=") < 0) payload += ",SRC=MQTT";
            enqueueIncoming(payload);
            publishStatusMsg(String("EVT|INQ|ENQ|SRC=MQTT"));
          }
        }
      }
    } else if (line.startsWith("+CMTI:")) {
      int comma = line.indexOf(','); if (comma>=0) {
        String idxStr = line.substring(comma+1); idxStr.trim(); int idx = idxStr.toInt();
        String sender; String body = modemReadSMSByIndex(idx, sender);
        if (body.length()) {
          String pl = body; if (pl.indexOf("SRC=") < 0) pl += ",SRC=SMS";
          pl += String(",_FROM=") + sender;
          enqueueIncoming(pl);
          publishStatusMsg(String("EVT|INQ|ENQ|SRC=SMS"));
        }
      }
    }
  }
}

// ---------- Token / source helpers ----------
String extractSrc(const String &payload) {
  int p = payload.indexOf("SRC=");
  if (p < 0) return String("UNKNOWN");
  String s = payload.substring(p + 4);
  int c = s.indexOf(',');
  if (c >= 0) s = s.substring(0, c);
  s.trim();
  return s;
}
String extractKeyVal(const String &payload, const String &key) {
  int p = payload.indexOf(key + "=");
  if (p < 0) return String("");
  String s = payload.substring(p + key.length() + 1);
  int c = s.indexOf(',');
  if (c >= 0) s = s.substring(0, c);
  s.trim();
  return s;
}
bool verifyTokenForSrc(const String &payload, const String &fromNumber = "") {
  String src = extractSrc(payload);
  if (src == "SMS") {
    if (fromNumber.length()) {
      if (isAdminNumber(fromNumber)) return true;
      String rec = extractKeyVal(payload, "RECOV");
      if (rec.length() && rec == sysConfig.recoveryTok) { Serial.println("Recovery token accepted for SMS from " + fromNumber); return true; }
      return false;
    }
    return false;
  }
  String tok = extractKeyVal(payload, "TOK");
  if (tok.length() && tok == sysConfig.sharedTok) return true;
  if (src == "BT") {
    String t2 = extractKeyVal(payload, "TOK_BT"); if (t2.length() && t2 == prefs.getString("tok_bt","")) return true;
  }
  if (src == "LORA") {
    String t2 = extractKeyVal(payload, "TOK_LORA"); if (t2.length() && t2 == prefs.getString("tok_lora","")) return true;
  }
  if (src == "MQTT") {
    String t2 = extractKeyVal(payload, "TOK_MQ"); if (t2.length() && t2 == prefs.getString("tok_mq","")) return true;
  }
  return false;
}


// ---------- Process queued incoming messages (concrete implementation) ----------
void processIncomingScheduleString(const String &payload) {
  String trimmed = payload; trimmed.trim();
  if (trimmed.length() == 0) return;
  String src = extractSrc(trimmed);
  String fromNumber = extractKeyVal(trimmed, "_FROM");
  Serial.printf("Processing incoming payload from %s : %s\n", src.c_str(), trimmed.c_str());

  // Auth check: for SMS we pass sender number
  if (!verifyTokenForSrc(trimmed, fromNumber)) {
    publishStatusMsg(String("ERR|AUTH_FAIL|SRC=") + src);
    Serial.println("Auth failed for payload: " + trimmed);
    return;
  }

  // If payload is JSON schedule
  if (trimmed.startsWith("{") || trimmed.startsWith("[")) {
    if (validateAndLoadScheduleFromJson(trimmed)) {
      broadcastStatus(String("EVT|SCH|SAVED|SRC=") + src);
    } else {
      publishStatusMsg("ERR|SCH|JSON_INVALID");
    }
    return;
  }

  // If compact schedule string (SCH|...)
  if (trimmed.indexOf("SCH|") >= 0) {
    if (saveCompactScheduleToMultipleFilesAndLoad(trimmed)) {
      broadcastStatus(String("EVT|SCH|SAVED|S=") + currentScheduleId + String("|SRC=") + src);
    } else {
      publishStatusMsg("ERR|SCH|INVALID");
    }
    return;
  }

  // Config as JSON via MQTT or compact CFG| via SMS/LORA/BT
  if (trimmed.indexOf("CFG|") >= 0) {
    // CFG|key=value,...
    String body = trimmed;
    int p = body.indexOf("CFG|");
    if (p >= 0) body = body.substring(p + 4);
    if (processSystemConfigSms(body, fromNumber)) {
      broadcastStatus(String("EVT|CFG|OK|SRC=") + src);
    } else publishStatusMsg("ERR|CFG|INVALID");
    return;
  }

  // MQTT/JSON config payload
  if (trimmed.startsWith("{") && trimmed.indexOf("MS")>=0) {
    if (processSystemConfigJson(trimmed)) {
      broadcastStatus(String("EVT|CFG|OK|SRC=") + src);
    } else publishStatusMsg("ERR|CFG|INVALID");
    return;
  }

  // If not recognized, log and respond
  Serial.println("Payload not recognized or unsupported format: " + trimmed);
  publishStatusMsg(String("ERR|UNKNOWN|SRC=") + src);
}
// ---------- Processing incoming queued messages ----------
void publishStatusMsg(const String &msg) {
  String out = msg; Serial.println("PublishStatus: " + out);
  if (mqttAvailable) { modemPublish(MQTT_TOPIC_STATUS, out); }
  if (ENABLE_SMS_BROADCAST) {
    ModemSerial.print("AT+CMGF=1\r\n"); delay(50);
    ModemSerial.print(String("AT+CMGS=\"") + sysConfig.adminPhones + String("\"\r\n")); delay(200);
    ModemSerial.print(out); ModemSerial.write(0x1A); delay(100);
  }
  sendLoRaCmdRaw(String("STAT|") + out);
}
void broadcastStatus(const String &msg) { publishStatusMsg(msg); }

// ---------- System config handlers ----------
bool processSystemConfigJson(const String &payload) {
  StaticJsonDocument<512> doc; DeserializationError err = deserializeJson(doc, payload);
  if (err) { Serial.printf("Config JSON parse error: %s\n", err.c_str()); return false; }
  if (doc.containsKey("MS")) sysConfig.mqttServer = String(doc["MS"].as<const char*>());
  if (doc.containsKey("MP")) sysConfig.mqttPort = doc["MP"].as<int>();
  if (doc.containsKey("MU")) sysConfig.mqttUser = String(doc["MU"].as<const char*>()); 
  if (doc.containsKey("MW")) sysConfig.mqttPass = String(doc["MW"].as<const char*>()); 
  if (doc.containsKey("ADMIN_PHONES") || doc.containsKey("AP")) {
    String aps = doc.containsKey("ADMIN_PHONES") ? String(doc["ADMIN_PHONES"].as<const char*>()) : String(doc["AP"].as<const char*>());
    String s = aps; s.trim(); sysConfig.adminPhones = "";
    int p = 0;
    while (p < (int)s.length()) {
      int c = s.indexOf(',', p);
      String part = (c==-1) ? s.substring(p) : s.substring(p, c); part.trim();
      if (part.length()) { String norm = normalizePhone(part); if (sysConfig.adminPhones.length()) sysConfig.adminPhones += ","; sysConfig.adminPhones += norm; }
      if (c == -1) break; p = c+1;
    }
  }
  if (doc.containsKey("SA")) sysConfig.simApn = String(doc["SA"].as<const char*>());
  if (doc.containsKey("SHARED_TOK")) sysConfig.sharedTok = String(doc["SHARED_TOK"].as<const char*>());
  if (doc.containsKey("RECOV")) sysConfig.recoveryTok = String(doc["RECOV"].as<const char*>());
  saveSystemConfig();
  publishStatusMsg("EVT|CFG|OK");
  return true;
}

bool processSystemConfigSms(const String &smsBody, const String &fromNumber) {
  String sender = normalizePhone(fromNumber);
  bool allowed = isAdminNumber(sender);
  String rec = extractKeyVal(smsBody, "RECOV");
  if (!allowed && rec.length() && rec == sysConfig.recoveryTok) {
    allowed = true;
    Serial.println("Recovery token used by " + sender);
  }
  if (!allowed) { Serial.printf("Unauthorized config SMS from %s ignored\n", sender.c_str()); return false; }
  String body = smsBody; if (body.startsWith("S|")) body = body.substring(2); body.trim();
  int pos = 0;
  while (pos < (int)body.length()) {
    int comma = body.indexOf(',', pos);
    String pair = (comma == -1)? body.substring(pos) : body.substring(pos, comma);
    int eq = pair.indexOf('='); if (eq > 0) {
      String key = pair.substring(0, eq); key.trim(); String val = pair.substring(eq+1); val.trim();
      if (key == "MS") sysConfig.mqttServer = val;
      else if (key == "MP") sysConfig.mqttPort = val.toInt();
      else if (key == "MU") sysConfig.mqttUser = val;
      else if (key == "MW") sysConfig.mqttPass = val;
      else if (key == "AP" || key == "ADMIN_PHONES") {
        String s = val; s.trim(); sysConfig.adminPhones = ""; int p = 0;
        while (p < (int)s.length()) {
          int c = s.indexOf(';', p); if (c==-1) c = s.indexOf(',', p);
          String part = (c==-1)? s.substring(p) : s.substring(p, c); part.trim();
          if (part.length()) { String norm = normalizePhone(part); if (sysConfig.adminPhones.length()) sysConfig.adminPhones += ","; sysConfig.adminPhones += norm; }
          if (c==-1) break; p = c+1;
        }
      }
      else if (key == "SA") sysConfig.simApn = val;
      else if (key == "LASTCLOSE_S") LAST_CLOSE_DELAY_MS = (uint32_t)(val.toInt() * 1000UL);
      else if (key == "DRIFT_S") DRIFT_THRESHOLD_S = (uint32_t)val.toInt();
      else if (key == "SYNC_H") { uint32_t h = (uint32_t)val.toInt(); if (h==0) h=1; SYNC_CHECK_INTERVAL_MS = h * 3600UL * 1000UL; }
      else if (key == "TOK") prefs.putString("tok_sms", val);
      else if (key == "TOK_LORA") prefs.putString("tok_lora", val);
      else if (key == "TOK_BT") prefs.putString("tok_bt", val);
      else if (key == "TOK_MQ") prefs.putString("tok_mq", val);
      else if (key == "RECOV") { sysConfig.recoveryTok = val; }
    }
    if (comma == -1) break;
    pos = comma + 1;
  }
  saveSystemConfig();
  publishStatusMsg("EVT|CFG|OK");
  return true;
}

// ---------- Compact parser & JSON schedule handling ----------
Schedule parseCompactSchedule(const String &payload) {
  Schedule s; s.id=""; s.rec='O'; s.start_epoch=0; s.timeStr=""; s.weekday_mask=0; s.seq.clear();
  s.pump_on_before_ms = PUMP_ON_LEAD_DEFAULT_MS; s.pump_off_after_ms = PUMP_OFF_DELAY_DEFAULT_MS; s.enabled=true; s.next_run_epoch=0; s.ts = 0;
  int p = payload.indexOf("SCH|"); String body = (p>=0)? payload.substring(p+4): payload; body.trim();
  int pos=0;
  while (pos < (int)body.length()) {
    int comma = body.indexOf(',', pos);
    String token = (comma==-1) ? body.substring(pos) : body.substring(pos, comma);
    token.trim();
    int eq = token.indexOf('=');
    if (eq>0) {
      String k = token.substring(0, eq); String v = token.substring(eq+1);
      k.trim(); v.trim();
      if (k == "ID") s.id = v;
      else if (k == "REC") s.rec = v.length()? v.charAt(0) : 'O';
      else if (k == "T") s.timeStr = v;
      else if (k == "SEQ") {
        String seqs = v; int spos=0;
        while (spos < (int)seqs.length()) {
          int semi = seqs.indexOf(';', spos);
          String pair = (semi==-1) ? seqs.substring(spos) : seqs.substring(spos, semi);
          int colon = pair.indexOf(':');
          if (colon>0) { SeqStep st; st.node_id = pair.substring(0, colon).toInt(); st.duration_ms = (uint32_t)pair.substring(colon+1).toInt() * 1000UL; s.seq.push_back(st); }
          if (semi==-1) break; spos = semi+1;
        }
      } else if (k == "WD") {
        String tmp = v; tmp.toUpperCase(); int sp=0;
        while (sp < (int)tmp.length()) {
          int cm = tmp.indexOf(',', sp);
          String d = (cm==-1)? tmp.substring(sp) : tmp.substring(sp, cm); d.trim();
          if (d=="MON") s.weekday_mask |= (1<<1); else if (d=="TUE") s.weekday_mask|=(1<<2); else if (d=="WED") s.weekday_mask|=(1<<3);
          else if (d=="THU") s.weekday_mask|=(1<<4); else if (d=="FRI") s.weekday_mask|=(1<<5); else if (d=="SAT") s.weekday_mask|=(1<<6); else if (d=="SUN") s.weekday_mask|=(1<<0);
          if (cm==-1) break; sp = cm+1;
        }
      } else if (k == "PB") s.pump_on_before_ms = (uint32_t)v.toInt();
      else if (k == "PA") s.pump_off_after_ms = (uint32_t)v.toInt();
      else if (k == "TS") s.ts = (uint32_t)v.toInt();
    }
    if (comma==-1) break; pos = comma+1;
  }
  if (s.rec == 'O' && s.timeStr.length()) {
    int year=0,mon=0,mday=0,hour=0,min=0,sec=0;
    if (sscanf(s.timeStr.c_str(), "%d-%d-%dT%d:%d:%d", &year, &mon, &mday, &hour, &min, &sec) >= 6) {
      struct tm tm; memset(&tm,0,sizeof(tm));
      tm.tm_year = year - 1900; tm.tm_mon = mon - 1; tm.tm_mday = mday; tm.tm_hour = hour; tm.tm_min = min; tm.tm_sec = sec;
      s.start_epoch = mktime(&tm);
    }
  }
  return s;
}

bool validateAndLoadScheduleFromJson(const String &json) {
  StaticJsonDocument<4096> scheduleDoc; scheduleDoc.clear();
  DeserializationError err = deserializeJson(scheduleDoc, json);
  if (err) { Serial.printf("JSON parse error: %s\n", err.c_str()); return false; }
  if (!scheduleDoc.containsKey("schedule_id") || !scheduleDoc.containsKey("sequence")) { Serial.println("JSON missing keys"); return false; }
  Schedule s; s.seq.clear();
  s.id = String((const char*)scheduleDoc["schedule_id"].as<const char*>());
  String recurrence = String((const char*)(scheduleDoc["recurrence"] | ""));
  if (recurrence.length()) { if (recurrence.startsWith("d")||recurrence.startsWith("D")) s.rec='D'; else if (recurrence.startsWith("w")||recurrence.startsWith("W")) s.rec='W'; else s.rec='O'; }
  else s.rec='O';
  s.timeStr = scheduleDoc.containsKey("start_time") ? String((const char*)scheduleDoc["start_time"].as<const char*>()) : "";
  s.start_epoch = scheduleDoc.containsKey("start_epoch") ? (time_t)(scheduleDoc["start_epoch"].as<long long>()) : 0;
  s.pump_on_before_ms = scheduleDoc.containsKey("pump_on_before_ms") ? scheduleDoc["pump_on_before_ms"].as<uint32_t>() : PUMP_ON_LEAD_DEFAULT_MS;
  s.pump_off_after_ms = scheduleDoc.containsKey("pump_off_after_ms") ? scheduleDoc["pump_off_after_ms"].as<uint32_t>() : PUMP_OFF_DELAY_DEFAULT_MS;
  s.ts = scheduleDoc.containsKey("ts") ? scheduleDoc["ts"].as<uint32_t>() : 0;
  if (scheduleDoc.containsKey("days") && scheduleDoc["days"].is<JsonArray>()) {
    s.weekday_mask = 0;
    for (JsonVariant v : scheduleDoc["days"].as<JsonArray>()) {
      String d = String((const char*)v.as<const char*>()); d.toUpperCase();
      if (d=="MON") s.weekday_mask |= (1<<1); else if (d=="TUE") s.weekday_mask |= (1<<2); else if (d=="WED") s.weekday_mask |= (1<<3);
      else if (d=="THU") s.weekday_mask |= (1<<4); else if (d=="FRI") s.weekday_mask |= (1<<5); else if (d=="SAT") s.weekday_mask |= (1<<6); else if (d=="SUN") s.weekday_mask |= (1<<0);
    }
  }
  JsonArray arr = scheduleDoc["sequence"].as<JsonArray>();
  for (JsonVariant v : arr) {
    SeqStep st; st.node_id = v["node_id"].as<int>(); if (v["duration_ms"]) st.duration_ms = v["duration_ms"].as<uint32_t>(); else if (v["duration_s"]) st.duration_ms = v["duration_s"].as<uint32_t>()*1000; else st.duration_ms = 0;
    s.seq.push_back(st);
  }
  if (!saveScheduleFile(s)) Serial.println("Warning: failed saving JSON schedule");
  bool found=false; for (size_t i=0;i<schedules.size();++i) if (schedules[i].id == s.id) { schedules[i]=s; found=true; break; }
  if (!found) schedules.push_back(s);
  if (!scheduleLoaded) { seq.clear(); for (auto &st: s.seq) seq.push_back(st); currentScheduleId = s.id; pumpOnBeforeMs = s.pump_on_before_ms; pumpOffAfterMs = s.pump_off_after_ms; scheduleLoaded=true; currentStepIndex=-1; scheduleStartEpoch = s.start_epoch; }
  return true;
}

bool saveCompactScheduleToMultipleFilesAndLoad(const String &compact) {
  Schedule s = parseCompactSchedule(compact);
  if (s.id.length() == 0) { Serial.println("Missing ID"); return false; }
  if (!saveScheduleFile(s)) Serial.println("Warning: failed saving schedule file");
  bool found=false; for (size_t i=0;i<schedules.size();++i) if (schedules[i].id == s.id) { schedules[i] = s; found=true; break; }
  if (!found) schedules.push_back(s);
  if (!scheduleLoaded) { seq.clear(); for (auto &st: s.seq) seq.push_back(st); currentScheduleId = s.id; pumpOnBeforeMs = s.pump_on_before_ms; pumpOffAfterMs = s.pump_off_after_ms; scheduleLoaded=true; currentStepIndex=-1; scheduleStartEpoch = s.start_epoch; }
  Serial.printf("Compact schedule saved id=%s seq=%d\n", s.id.c_str(), (int)s.seq.size());
  return true;
}

// ---------- Scheduler helpers ----------
bool parseTimeHHMM(const String &t, int &hour, int &minute) {
  hour = 0; minute = 0; int res = sscanf(t.c_str(), "%d:%d", &hour, &minute); return res==2;
}
time_t nextWeekdayOccurrence(time_t now, uint8_t weekday_mask, int hour, int minute) {
  struct tm tmnow; localtime_r(&now,&tmnow); int today = tmnow.tm_wday;
  for (int d=0; d<14; ++d) {
    int day = (today + d) % 7;
    if (weekday_mask & (1<<day)) {
      struct tm tmCandidate = tmnow; tmCandidate.tm_mday += d; tmCandidate.tm_hour = hour; tmCandidate.tm_min = minute; tmCandidate.tm_sec = 0;
      time_t cand = mktime(&tmCandidate);
      if (cand > now) return cand;
    }
  }
  return 0;
}
time_t computeNextRunEpoch(const Schedule &s, time_t now) {
  if (!s.enabled) return 0;
  if (s.rec == 'O') return s.start_epoch;
  else if (s.rec == 'D') {
    int hh,mm; if (!parseTimeHHMM(s.timeStr, hh, mm)) return 0;
    struct tm tmnow; localtime_r(&now,&tmnow);
    struct tm tmc = tmnow; tmc.tm_hour = hh; tmc.tm_min = mm; tmc.tm_sec = 0;
    time_t cand = mktime(&tmc); if (cand > now) return cand;
    tmc.tm_mday += 1; return mktime(&tmc);
  } else if (s.rec == 'W') {
    int hh,mm; if (!parseTimeHHMM(s.timeStr, hh, mm)) return 0;
    return nextWeekdayOccurrence(now, s.weekday_mask, hh, mm);
  }
  return 0;
}

// ---------- Broadcast & status ---------- (already implemented above)

// ---------- Scheduler execution ----------
void setPump(bool on) {
  pinMode(PUMP_PIN, OUTPUT);
  if (PUMP_ACTIVE_HIGH) digitalWrite(PUMP_PIN, on?HIGH:LOW); else digitalWrite(PUMP_PIN, on?LOW:HIGH);
  Serial.printf("Pump %s\n", on?"ON":"OFF");
}

void startScheduleIfDue() {
  if (!scheduleLoaded) return;
  if (scheduleRunning) return;
  if (seq.size()==0) return;
  time_t now = time(nullptr); if (now == (time_t)-1) return;
  int startIndex = -1;
  for (size_t i=0;i<seq.size();++i) {
    int node = seq[i].node_id;
    uint32_t dur = seq[i].duration_ms;
    Serial.printf("Attempt OPEN idx %u node %d\n", (unsigned)i, node);
    if (sendCmdWithAck("OPEN", node, currentScheduleId, (int)i, dur)) { startIndex = (int)i; break; }
  }
  if (startIndex < 0) { publishStatusMsg("ERR|no_start_node_opened"); return; }
  for (size_t i=0;i<seq.size();++i) {
    if ((int)i == startIndex) continue;
    sendCmdWithAck("CLOSE", seq[i].node_id, currentScheduleId, (int)i, 0);
  }
  setPump(true); delay(pumpOnBeforeMs);
  scheduleRunning = true; currentStepIndex = startIndex; stepStartMillis = millis();
  prefs.putInt("active_index", currentStepIndex);
  publishStatusMsg(String("EVT|START|S=") + currentScheduleId);
}

void stopScheduleAndCleanup() {
  if (currentStepIndex >=0 && currentStepIndex < (int)seq.size()) sendCmdWithAck("CLOSE", seq[currentStepIndex].node_id, currentScheduleId, currentStepIndex, 0);
  setPump(false); scheduleRunning=false; currentStepIndex=-1; prefs.putInt("active_index", currentStepIndex); publishStatusMsg("EVT|SCHEDULE_STOPPED");
}

void runScheduleLoop() {
  if (!scheduleRunning) { startScheduleIfDue(); return; }
  if (currentStepIndex < 0 || currentStepIndex >= (int)seq.size()) {
    delay(pumpOffAfterMs);
    setPump(false);
    scheduleRunning=false;
    publishStatusMsg("EVT|SCHEDULE_COMPLETE");
    currentStepIndex = -1; prefs.putInt("active_index", currentStepIndex); return;
  }
  SeqStep &step = seq[currentStepIndex];
  if (millis() - stepStartMillis >= step.duration_ms) {
    int nextIdx = -1;
    for (int cand = currentStepIndex+1; cand < (int)seq.size(); ++cand) {
      if (sendCmdWithAck("OPEN", seq[cand].node_id, currentScheduleId, cand, seq[cand].duration_ms)) { nextIdx = cand; break; }
    }
    sendCmdWithAck("CLOSE", step.node_id, currentScheduleId, currentStepIndex, 0);
    if (nextIdx >= 0) {
      currentStepIndex = nextIdx; prefs.putInt("active_index", currentStepIndex); stepStartMillis = millis(); publishStatusMsg(String("EVT|STEP|MOVE|I=")+String(currentStepIndex));
    } else {
      setPump(false); scheduleRunning=false; publishStatusMsg("EVT|SCHEDULE_COMPLETE|NO_NEXT");
      currentStepIndex = -1; prefs.putInt("active_index", currentStepIndex);
    }
  }
  if (millis() - lastProgressSave > SAVE_PROGRESS_INTERVAL_MS) {
    prefs.putString("active_schedule", currentScheduleId);
    prefs.putInt("active_index", currentStepIndex);
    lastProgressSave = millis();
  }
}

// ---------- NTP/RTC ----------
bool connectWiFiOnce() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.mode(WIFI_STA); WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) { delay(200); Serial.print("."); }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}
void disconnectWiFiOnce() { WiFi.disconnect(true); WiFi.mode(WIFI_OFF); delay(100); }

bool oneShotNtpSyncAndSetRTC() {
  if (!connectWiFiOnce()) return false;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo; unsigned long tstart = millis(); bool gotTime=false;
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
  DateTime rtcTime = rtc.now(); time_t rtcEpoch = rtcTime.unixtime();
  time_t sysEpoch = time(nullptr);
  if (sysEpoch <= 0) { oneShotNtpSyncAndSetRTC(); return; }
  long diff = (long)sysEpoch - (long)rtcEpoch; long absdrift = diff>=0?diff:-diff;
  if ((uint32_t)absdrift > DRIFT_THRESHOLD_S) {
    if (oneShotNtpSyncAndSetRTC()) publishStatusMsg("EVT|NTP_SYNC|OK"); else publishStatusMsg("ERR|NTP_SYNC_FAIL");
  }
}

// ---------- Setup & Loop ----------
void initBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(BLE_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setValue("OK"); pCharacteristic->setCallbacks(new ControllerBLECallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); pAdvertising->addServiceUUID(BLE_SERVICE_UUID); pAdvertising->start();
  Serial.println("BLE started");
}

void setup() {
  Serial.begin(115200); delay(200);
  initStorage(); prefs.begin("irrig", false);
  displayInitHeltec();
  loadSystemConfig();
  if (!LittleFS.exists("/schedules")) LittleFS.mkdir("/schedules");
  loadAllSchedulesFromFS();

  // Initialize dedicated RTC I2C bus
  WireRTC.begin(RTC_SDA, RTC_SCL, 100000); // SDA=41, SCL=42, 100kHz
  delay(20);
  rtcAvailable = rtc.begin(&WireRTC);  // use custom I2C for DS3231
  if (rtcAvailable) {
    Serial.println("RTC detected on WireRTC");
    if (rtc.lostPower()) {
      Serial.println("RTC lost power; setting from compile time");
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  } else {
    Serial.println("RTC not detected on WireRTC bus");
  }

  loraInit();
  modemInit();
  modemConfigureAndConnectMQTT();
  initBLE();
  lastStatusPublish = millis();
  Serial.println("Setup complete");
}

unsigned long lastSchedulerCheck = 0;
void loop() {
  modemBackgroundRead();
  handleLoRaIncoming();
  // process one queued incoming message
  String iq; if (dequeueIncoming(iq)) { Serial.println("Processing queued incoming: " + iq); processIncomingScheduleString(iq); }
  runScheduleLoop();
  if (millis() - lastSchedulerCheck > 5000) {
    time_t now = time(nullptr);
    for (auto &sch : schedules) {
      if (!sch.enabled) continue;
      if (sch.next_run_epoch == 0) sch.next_run_epoch = computeNextRunEpoch(sch, now);
      if (sch.next_run_epoch > 0 && now >= sch.next_run_epoch) {
        currentScheduleId = sch.id; seq.clear(); for (auto &st: sch.seq) seq.push_back(st);
        pumpOnBeforeMs = sch.pump_on_before_ms; pumpOffAfterMs = sch.pump_off_after_ms;
        scheduleStartEpoch = sch.next_run_epoch; scheduleLoaded = true; currentStepIndex = -1;
        publishStatusMsg(String("EVT|SCH|TRIGGER|S=") + sch.id);
        if (sch.rec == 'O') sch.enabled = false;
        sch.next_run_epoch = computeNextRunEpoch(sch, now + 1);
        break;
      }
    }
    lastSchedulerCheck = millis();
  }
  checkRtcDriftAndSync();
  if (millis() - lastStatusPublish > statusPublishInterval) { publishStatusMsg(String("EVT|RUN|S=") + (scheduleRunning?String("1"):String("0"))); lastStatusPublish = millis(); }
  displayLoop();
  delay(20);
}

/* End of file */
