/* 
  Main Controller - Final (Heltec LoRa) - PATCHED for SMS sending
  - Changes: ENABLE_SMS_BROADCAST enabled, robust SMS helpers added
  - Added: modemReadyForSMS(), waitForPrompt(), sendSMS(), updated publishStatusMsg to send SMS per-admin
*/

#include "LoRaWan_APP.h"
#include "Arduino.h"
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
#include <BLE2902.h> // for notifications descriptor

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
#define DEFAULT_ADMIN_PHONE "+919944272647"

#define MQTT_TOPIC_SCHEDULE "irrigation/site01/schedule/set"
#define MQTT_TOPIC_CONFIG   "irrigation/site01/config/system/set"
#define MQTT_TOPIC_STATUS   "irrigation/site01/status"

// LoRa SPI pins
#define LORA_CS   18
#define LORA_RST  14
#define LORA_DIO0 26
// LoRa freq as long (Hz). Use PABOOST true for higher power if your board/antenna supports it.
#define LORA_FREQ 865000000L  // 865 MHz
// --- Heltec Radio (LoRaWan_APP) Parameters ---
#define RF_FREQUENCY         865000000 // Hz
#define TX_OUTPUT_POWER      5         // dBm
#define LORA_BANDWIDTH       0         // [0:125kHz,1:250kHz,2:500kHz]
#define LORA_SPREADING_FACTOR 7        // [SF7..SF12]
#define LORA_CODINGRATE      1         // [1:4/5,2:4/6,3:4/7,4:4/8]
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT  0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT_VALUE     1000
#define BUFFER_SIZE          64


// Controller pump pin
#define PUMP_PIN 47
#define PUMP_ACTIVE_HIGH true

// BLE
#define BLE_DEVICE_NAME "IrrigCtrl"
#define BLE_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
BLEServer *pServer = nullptr;
BLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

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
bool ENABLE_SMS_BROADCAST = true; // enabled: allow SMS fallback/broadcast
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

// ---- Manual mode globals ----
bool manualMode = false;                    // true => manual mode active (schedules disabled)
const char* PREF_MANUAL_MODE = "manual_mode";
unsigned long manualModeLastActive = 0;     // millis() of last manual activity
unsigned long MANUAL_INACTIVITY_MS = 0;     // 0 = disabled (persisted)
const char* PREF_MANUAL_TIMEOUT_MS = "manual_to_ms";


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

void VextON(){ pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }
void VextOFF(){ pinMode(Vext, OUTPUT); digitalWrite(Vext, HIGH); }

// ---------- Display helpers ----------
void displayInitHeltec() {
  VextON(); 
  delay(50);
  display.init();
  display.setFont(ArialMT_Plain_10);
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "Irrigation Controller");
  display.drawString(0, 12, "Booting...");
  display.display();
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
  d["schedule_id"] = s.id; 
  d["recurrence"] = (s.rec=='D'?"daily":(s.rec=='W'?"weekly":"onetime"));
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
  sysConfig.adminPhones = prefs.getString("admin_phones", DEFAULT_ADMIN_PHONE);
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

// ---------- MODEM helpers ----------
String sendAT(const String &cmd, unsigned long timeoutMs = 2000) {
  while (ModemSerial.available()) ModemSerial.read();
  if (cmd.length()) ModemSerial.print(cmd + String("\r\n"));
  unsigned long start = millis();
  String out;
  while (millis() - start < timeoutMs) {
    while (ModemSerial.available()) { char c = (char)ModemSerial.read(); out += c; lastModemActivity = millis(); }
    delay(5);
  }
  return out;
}

// Wait for a specific prompt character (eg '>' ) from modem
bool waitForPrompt(char ch, unsigned long timeout = 5000) {
  unsigned long start = millis();
  String buf;
  while (millis() - start < timeout) {
    while (ModemSerial.available()) {
      char c = (char)ModemSerial.read(); buf += c; lastModemActivity = millis();
      if (c == ch) return true;
    }
    delay(5);
  }
  return false;
}

// -------------------- Modem NTP sync (AT+QNTP) --------------------
// Attempts to sync via the modem's NTP command (AT+QNTP) and set ESP32 time + RTC.
// Returns true on success.
bool modemNtpSyncAndSetRTC() {
  Serial.println("Attempting modem NTP sync via AT+QNTP...");

  // Ensure PDP active (harmless if already active)
  String setPdp = String("AT+QICSGP=1,1,\"") + sysConfig.simApn + String("\",\"\",\"\",1");
  sendAT(setPdp, 4000);
  sendAT("AT+QIACT=1", 10000);

  // Issue the NTP command. Many modems return a URC like "+QNTP: ...", we read for a while.
  String resp = sendAT(String("AT+QNTP=1,\"") + ntpServer + String("\""), 15000);
  Serial.println("AT+QNTP resp:\n" + resp);

  // Look for a +QNTP: line in the response (URC) or any timestamp-like substring
  int qpos = resp.indexOf("+QNTP:");
  String dtStr = "";

  if (qpos >= 0) {
    // get the line containing +QNTP:
    int lineEnd = resp.indexOf('\n', qpos);
    if (lineEnd < 0) lineEnd = resp.length();
    String line = resp.substring(qpos, lineEnd);
    // try to extract a date/time segment - search for year '20'
    int ypos = line.indexOf("20");
    if (ypos >= 0) dtStr = line.substring(ypos);
    else {
      // fallback: use entire line after colon
      int colon = line.indexOf(':');
      if (colon >= 0) dtStr = line.substring(colon + 1);
      else dtStr = line;
    }
  } else {
    // fallback: attempt to find a yyyy/mm/dd or yyyy-mm-dd pattern anywhere
    int ypos = resp.indexOf("20");
    if (ypos >= 0) dtStr = resp.substring(ypos);
  }

  dtStr.trim();
  Serial.println("Parsed candidate date string: '" + dtStr + "'");

  if (dtStr.length() == 0) {
    Serial.println("modemNtpSync: no datetime substring found in +QNTP response");
    return false;
  }

  // Try multiple common formats tolerant parsing
  int year=0, mon=0, mday=0, hour=0, minute=0, second=0;
  bool parsed = false;

  // Format 1: YYYY/MM/DD,HH:MM:SS  (some modems)
  if (!parsed) {
    if (sscanf(dtStr.c_str(), "%d/%d/%d,%d:%d:%d", &year, &mon, &mday, &hour, &minute, &second) == 6) parsed = true;
  }
  // Format 2: YYYY-MM-DD HH:MM:SS
  if (!parsed) {
    if (sscanf(dtStr.c_str(), "%d-%d-%d %d:%d:%d", &year, &mon, &mday, &hour, &minute, &second) == 6) parsed = true;
  }
  // Format 3: YYYY/MM/DD HH:MM:SS
  if (!parsed) {
    if (sscanf(dtStr.c_str(), "%d/%d/%d %d:%d:%d", &year, &mon, &mday, &hour, &minute, &second) == 6) parsed = true;
  }
  // Format 4: YYYYMMDDHHMMSS (continuous digits)
  if (!parsed) {
    if ((int)dtStr.length() >= 14) {
      const char *p = dtStr.c_str();
      if (isdigit(p[0]) && isdigit(p[13])) {
        char buf[5];
        memcpy(buf, p, 4); buf[4]=0; year = atoi(buf);
        memcpy(buf, p+4, 2); buf[2]=0; mon = atoi(buf);
        memcpy(buf, p+6, 2); buf[2]=0; mday = atoi(buf);
        memcpy(buf, p+8, 2); buf[2]=0; hour = atoi(buf);
        memcpy(buf, p+10, 2); buf[2]=0; minute = atoi(buf);
        memcpy(buf, p+12, 2); buf[2]=0; second = atoi(buf);
        parsed = (year>2000 && mon>=1 && mon<=12 && mday>=1 && mday<=31);
      }
    }
  }
  // Format 5: try to scan any integers in order: y m d h m s
  if (!parsed) {
    // collect integers
    std::vector<int> nums;
    String tmp = dtStr;
    tmp.replace('/', ' '); tmp.replace('-', ' '); tmp.replace(':', ' '); tmp.replace(',', ' ');
    const char *c = tmp.c_str();
    int val = 0; bool innum = false;
    for (int i=0; i<(int)tmp.length(); ++i) {
      if (isdigit(c[i])) { innum = true; val = val*10 + (c[i]-'0'); }
      else { if (innum) { nums.push_back(val); val=0; innum=false; } }
    }
    if (innum) nums.push_back(val);
    if ((int)nums.size() >= 6) {
      year = nums[0]; mon = nums[1]; mday = nums[2]; hour = nums[3]; minute = nums[4]; second = nums[5];
      parsed = (year>2000 && mon>=1 && mon<=12);
    }
  }

  if (!parsed) {
    Serial.println("modemNtpSync: failed to parse date/time from modem response");
    return false;
  }

  // Assemble tm structure and set system time (treat as local time -> convert using gmtOffset)
  struct tm tmnow;
  memset(&tmnow, 0, sizeof(tmnow));
  tmnow.tm_year = year - 1900;
  tmnow.tm_mon  = mon - 1;
  tmnow.tm_mday = mday;
  tmnow.tm_hour = hour;
  tmnow.tm_min  = minute;
  tmnow.tm_sec  = second;

  // mktime assumes localtime; we want epoch in UTC relative to timezone offsets.
  time_t t = mktime(&tmnow);
  if (t == (time_t)-1) {
    Serial.println("modemNtpSync: mktime failed");
    return false;
  }

  // set system time (adjust for gmtOffset)
  // The modem's returned time is almost always UTC or local — many EC200U returns UTC.
  // We'll assume the returned time is UTC. If needed, adjust here.
  struct timeval tv;
  tv.tv_sec = t;
  tv.tv_usec = 0;
  if (settimeofday(&tv, NULL) != 0) {
    Serial.println("modemNtpSync: settimeofday failed");
    // still try RTC update below
  } else {
    Serial.printf("System time set from modem: %s", ctime(&t));
  }

  // update DS3231 if available
  if (rtcAvailable) {
    DateTime dt((uint32_t)t);
    rtc.adjust(dt);
    Serial.println("RTC updated from modem NTP.");
  }

  prefs.putULong("last_ntp_sync", (unsigned long)t);
  return true;
}

// Basic readiness checks before attempting SMS
bool modemReadyForSMS() {
  String creg = sendAT("AT+CREG?", 2000);
  if (creg.indexOf(",1") < 0 && creg.indexOf(",5") < 0) return false;
  String cpin = sendAT("AT+CPIN?", 2000);
  if (cpin.indexOf("READY") < 0) return false;
  return true;
}

void modemInit(){
  ModemSerial.begin(MODEM_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(200);
  while (ModemSerial.available()) ModemSerial.read();
  Serial.println("Modem serial init");
  // Configure text-mode and new message indications so +CMTI is emitted
  sendAT("AT+CMGF=1", 1000);
  sendAT("AT+CSCS=\"GSM\"", 1000);
  sendAT("AT+CNMI=2,1,0,0,0", 1000);
  sendAT("AT+QURCCFG=\"urcport\",\"uart1\""); // ensure URCs on UART1
  sendAT("AT+QCFG=\"urc/ri/smsincoming\"");   // query smsincoming config
}

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

// Send SMS to a single number, robustly waiting for > and reading response
bool sendSMS(const String &num, const String &text) {
  if (!modemReadyForSMS()) {
    Serial.println("Modem not ready for SMS (no network or SIM locked)");
    return false;
  }
  sendAT("AT+CMGF=1", 1000);
  sendAT("AT+CSCS=\"GSM\"", 1000);
  String cmd = String("AT+CMGS=\"") + num + String("\"");
  // Clear any pending chars
  while (ModemSerial.available()) ModemSerial.read();
  ModemSerial.print(cmd);
  ModemSerial.print("\r\n");
  if (!waitForPrompt('>', 7000)) {
    Serial.println("No > prompt received from modem for CMGS");
    String dump = sendAT("", 200);
    Serial.println("CMGS dump:" + dump);
    return false;
  }
  // send body and Ctrl+Z
  ModemSerial.print(text);
  ModemSerial.write(0x1A);
  // wait for response (may take several seconds)
  String resp = sendAT("", 10000);
  Serial.println("SMS send resp: " + resp);
  if (resp.indexOf("+CMGS:") >= 0 || resp.indexOf("OK") >= 0) return true;
  return false;
}

// ---------- LoRa helpers ----------
// ---------- LoRa (Radio driver) ----------
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

void loraInit() {
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone    = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  Serial.println("Heltec Radio LoRa init OK");
}
void OnTxDone(void) {
  Serial.println("[Radio] TX done");
  Radio.Rx(0);  // switch back to receive mode
}

void OnTxTimeout(void) {
  Serial.println("[Radio] TX timeout");
  Radio.Rx(0);
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Serial.printf("[Radio] RX %d bytes RSSI=%d SNR=%d => %s\n", size, rssi, snr, rxpacket);

  // handle incoming LoRa message like before
  String msg = String(rxpacket);
  msg.trim();
  if (msg.length()) {
    enqueueIncoming(msg);
    publishStatusMsg(String("EVT|INQ|ENQ|SRC=LORA"));
  }

  // return to RX mode
  Radio.Rx(0);
}


void sendLoRaCmdRaw(const String &cmd) {
  snprintf(txpacket, BUFFER_SIZE, "%s", cmd.c_str());
  Radio.Send((uint8_t *)txpacket, strlen(txpacket));
  Serial.printf("[Radio] TX: %s\n", txpacket);
}

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
      int q1 = header.indexOf('"');
      if (q1 >= 0) {
        int q2 = header.indexOf('"', q1 + 1);
        int q3 = header.indexOf('"', q2 + 1);
        int q4 = header.indexOf('"', q3 + 1);
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
      int firstQuote = line.indexOf('"');
      if (firstQuote>=0) {
        int secondQuote = line.indexOf('"', firstQuote+1);
        if (secondQuote>firstQuote) {
          int thirdQuote = line.indexOf('"', secondQuote+1);
          int fourthQuote = line.indexOf('"', thirdQuote+1);
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

// ---------- Modified publish / broadcast that sends SMS per-admin ----------
void publishStatusMsg(const String &msg) {
  String out = msg;
  Serial.println("PublishStatus: " + out);

  // 1) MQTT (if available)
  if (mqttAvailable) { modemPublish(MQTT_TOPIC_STATUS, out); }

  // 2) BLE notify (if client connected and TX char exists)
  if (deviceConnected && pTxCharacteristic != nullptr) {
    // Build a compact BLE notification payload (avoid huge messages)
    String btMsg = String("STAT|") + out;
    if (btMsg.length() > 200) btMsg = btMsg.substring(0, 200);
    pTxCharacteristic->setValue((uint8_t*)btMsg.c_str(), btMsg.length());
    pTxCharacteristic->notify();
    Serial.print("BLE notify sent: ");
    Serial.println(btMsg);
    delay(10); // short breathing room for BLE stack
  }

  // 3) SMS fallback (if enabled)
  if (ENABLE_SMS_BROADCAST) {
    auto admins = adminPhoneList();
    if (admins.size() == 0) {
      String single = sysConfig.adminPhones;
      if (single.length()) {
        if (sendSMS(single, out)) Serial.println("SMS sent to fallback admin string");
        else Serial.println("SMS failed to fallback admin string");
      }
    } else {
      for (auto &num : admins) {
        String n = normalizePhone(num);
        Serial.println(String("Sending SMS to ") + n + ": " + out);
        if (modemReadyForSMS()) {
          if (sendSMS(n, out)) Serial.println("SMS OK to " + n);
          else Serial.println("SMS FAILED to " + n);
        } else {
          Serial.println("Modem not ready for SMS (skipping): " + n);
        }
        delay(500);
      }
    }
  }
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
            else if (key == "MODE") {
        String v = val; v.trim(); v.toUpperCase();
        if (v == "MAN" || v == "MANUAL") enterManualMode();
        else exitManualMode();
      }
      else if (key == "EMERGENCY") {
        String v = val; v.trim(); v.toUpperCase();
        if (v == "STOP") emergencyStopAll();
      }
      else if (key == "MANUAL_CMD") {
        // compact manual commands: "PUMP=ON" or "VALVE=2:1:OPEN" or "STATUS" or "TIMEOUT_MS=60000"
        String cmd = val; cmd.trim(); cmd.toUpperCase();
        // ensure manual mode active
        enterManualMode();
        setManualActivity();
        if (cmd.startsWith("PUMP=")) {
          String p = cmd.substring(5);
          if (p == "ON") { setPump(true); publishStatusIfAvailable("ACK|MANUAL|PUMP|ON"); }
          else { setPump(false); publishStatusIfAvailable("ACK|MANUAL|PUMP|OFF"); }
        } else if (cmd.startsWith("VALVE=")) {
          // format VALVE=node:valve:OPEN
          String param = cmd.substring(6);
          int p1 = param.indexOf(':');
          int p2 = param.indexOf(':', p1+1);
          if (p1>0 && p2>p1) {
            int node = param.substring(0,p1).toInt();
            int valve = param.substring(p1+1, p2).toInt();
            String action = param.substring(p2+1);
            if (action == "OPEN") { sendCmdWithAck("OPEN", node, currentScheduleId, valve, 0); publishStatusIfAvailable("ACK|MANUAL|VALVE|OPEN"); }
            else { sendCmdWithAck("CLOSE", node, currentScheduleId, valve, 0); publishStatusIfAvailable("ACK|MANUAL|VALVE|CLOSE"); }
          } else publishStatusIfAvailable("ERR|MANUAL|VALVE|BAD_FORMAT");
        } else if (cmd == "STATUS") {
          publishStatusIfAvailable(String("STATUS|MODE|") + (manualMode ? "MANUAL" : "SCHEDULE") + String("|RUNNING|") + (scheduleRunning ? "1":"0"));
        } else if (cmd.startsWith("TIMEOUT_MS=")) {
          unsigned long t = (unsigned long) atol(cmd.substring(11).c_str());
          MANUAL_INACTIVITY_MS = t;
          prefs.putULong(PREF_MANUAL_TIMEOUT_MS, MANUAL_INACTIVITY_MS);
          publishStatusIfAvailable(String("ACK|MANUAL|TIMEOUT_MS|") + String(t));
        } else {
          publishStatusIfAvailable("ERR|MANUAL|CMD_UNKNOWN");
        }
      }
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

// ---- Manual mode helpers ----
void setManualActivity() {
  manualModeLastActive = millis();
}

// wrapper to reuse publish path
void publishStatusIfAvailable(const String &s) {
  publishStatusMsg(s);
}

// Enter manual mode: stop any running schedule cleanly (respect pumpOffAfterMs & LAST_CLOSE_DELAY_MS)
void enterManualMode() {
  if (manualMode) return;
  Serial.println("Switching to MANUAL mode");
  publishStatusIfAvailable("EVT|MODE|MANUAL");

  // If a schedule is running, stop it cleanly
  if (scheduleRunning) {
    publishStatusIfAvailable("EVT|MANUAL_OVERRIDE|STOPPING");
    // wait pump off lead time then stop pump
    delay(pumpOffAfterMs);
    setPump(false);
    // Wait LAST_CLOSE_DELAY_MS while servicing modem/display (avoid blocking forever)
    unsigned long waitStart = millis();
    while (millis() - waitStart < LAST_CLOSE_DELAY_MS) {
      modemBackgroundRead();
      displayLoop();
      delay(50);
    }
    // close currently open valve (best-effort)
    if (currentStepIndex >= 0 && currentStepIndex < (int)seq.size()) {
      int lastNode = seq[currentStepIndex].node_id;
      bool ok = sendCmdWithAck("CLOSE", lastNode, currentScheduleId, currentStepIndex, 0);
      if (!ok) Serial.println("WARN: manual close ACK failed");
    }
    scheduleRunning = false;
    currentStepIndex = -1;
    prefs.putInt("active_index", currentStepIndex);
    publishStatusIfAvailable("EVT|MANUAL_OVERRIDE|STOPPED");
  }

  manualMode = true;
  prefs.putBool(PREF_MANUAL_MODE, true);
  setManualActivity();
  // update display quickly
  disp_status_line = "MODE: MANUAL";
  displayLoop();
}

// Exit manual mode (do NOT auto-start schedules)
void exitManualMode() {
  if (!manualMode) return;
  Serial.println("Switching to SCHEDULE mode");
  publishStatusIfAvailable("EVT|MODE|SCHEDULE");
  manualMode = false;
  prefs.putBool(PREF_MANUAL_MODE, false);
  disp_status_line = "MODE: SCHEDULE";
  displayLoop();
}

// Immediate emergency stop (no delays): close valves and stop pump
void emergencyStopAll() {
  Serial.println("EMERGENCY STOP: immediate");
  publishStatusIfAvailable("EVT|EMERGENCY_STOP|START");
  #ifdef VALVE_PINS
    for (int i = 0; i < NUM_VALVES; ++i) {
      if (VALVE_PINS[i] >= 0) {
        pinMode(VALVE_PINS[i], OUTPUT);
        if (VALVE_ACTIVE_HIGH[i]) digitalWrite(VALVE_PINS[i], LOW);
        else digitalWrite(VALVE_PINS[i], HIGH);
      }
    }
  #else
    // best-effort: close all nodes referenced by seq
    for (size_t i = 0; i < seq.size(); ++i) {
      sendCmdWithAck("CLOSE", seq[i].node_id, currentScheduleId, (int)i, 0);
    }
  #endif
  setPump(false);
  scheduleRunning = false;
  currentStepIndex = -1;
  prefs.putInt("active_index", currentStepIndex);
  publishStatusIfAvailable("EVT|EMERGENCY_STOP|DONE");
}
void manualInactivityCheck() {
  if (!manualMode) return;
  if (MANUAL_INACTIVITY_MS == 0) return;
  if ((millis() - manualModeLastActive) > MANUAL_INACTIVITY_MS) {
    Serial.println("Manual inactivity timeout reached - exiting manual mode");
    publishStatusIfAvailable("EVT|MODE|MANUAL_TIMEOUT_EXIT");
    exitManualMode();
  }
}


void startScheduleIfDue() {
   if (manualMode) {
    Serial.println("Manual mode active; not starting schedule");
    return;
  }
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
  // If manual mode engaged while scheduleRunning, halt schedule progression
  if (manualMode && scheduleRunning) {
    Serial.println("runScheduleLoop: manualMode active, halting schedule progression");
    return;
  }
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
  // Try modem first (no WiFi required)
  if (modemNtpSyncAndSetRTC()) {
    Serial.println("NTP sync via modem OK");
    publishStatusMsg("EVT|NTP_SYNC|MODEM_OK");
    return true;
  }

  Serial.println("Modem NTP failed; trying WiFi fallback...");

  if (!connectWiFiOnce()) return false;
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  struct tm timeinfo; unsigned long tstart = millis(); bool gotTime = false;
  while ((millis() - tstart) < NTP_TIMEOUT_MS) {
    if (getLocalTime(&timeinfo, 2000)) { gotTime = true; break; }
    delay(200);
  }
  if (!gotTime) { disconnectWiFiOnce(); return false; }
  time_t now = time(nullptr);
  DateTime dt((uint32_t)now);
  if (rtcAvailable) rtc.adjust(dt);
  prefs.putULong("last_ntp_sync", (unsigned long)now);
  disconnectWiFiOnce();
  publishStatusMsg("EVT|NTP_SYNC|WIFI_OK");
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

// BLE callbacks
// ---- Replace existing ControllerBLECallbacks with this corrected handler ----
class ControllerBLECallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    // Use auto to accept either std::string or Arduino String and convert safely
    auto v = pChar->getValue();
    String payload = String(v.c_str());   // robust conversion
    payload.trim();
    Serial.print("BLE_RX: ");
    Serial.println(payload);

    if (payload.length() == 0) return;

    // ensure SRC tag present for later processing
    if (payload.indexOf("SRC=") < 0) payload += String(",SRC=BT");

    // enqueue and process normally
    enqueueIncoming(payload);
    publishStatusMsg(String("EVT|INQ|ENQ|SRC=BT"));

    // Build an acknowledgment. Prefer to echo MID if present.
    String mid = extractKeyVal(payload, "MID"); // uses your existing helper
    String ack;
    if (mid.length()) {
      ack = String("ACK|MID=") + mid + String("|BT|OK");
    } else {
      // no MID — send a simple echo ack for debugging
      ack = String("ACK|BT|ECHO|") + payload;
      if (ack.length() > 200) ack = ack.substring(0, 200);
    }

    // Send notification back if TX characteristic exists and a client is connected
    if (pTxCharacteristic != nullptr && deviceConnected) {
      // setValue accepts either std::string or byte buffer; use bytes for safety
      pTxCharacteristic->setValue((uint8_t *)ack.c_str(), ack.length());
      pTxCharacteristic->notify();
      Serial.print("BLE_TX (notify): ");
      Serial.println(ack);
      delay(10);
    } else {
      Serial.println("BLE_TX: cannot notify - no client or TX char null");
    }
  }
};


// ---- Add this BLE server callback to track connection state ----
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("BLE: client connected");
    // optionally stop advertising to be less chatty
    BLEAdvertising *adv = BLEDevice::getAdvertising();
    if (adv) adv->stop();
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("BLE: client disconnected");
    // restart advertising to allow new connections
    BLEDevice::startAdvertising();
  }
};

// ---- Replace your initBLE() with this improved version ----
void initBLE() {
  Serial.println("initBLE: starting");
  BLEDevice::init(BLE_DEVICE_NAME); // name shown by phone

  // create server and attach callbacks
  pServer = BLEDevice::createServer();
  if (!pServer) {
    Serial.println("initBLE: ERROR createServer() returned NULL");
    return;
  }
  pServer->setCallbacks(new MyServerCallbacks());

  // use UART-like service/characteristics (you already defined SERVICE_UUID etc.)
  BLEService *pService = pServer->createService(SERVICE_UUID);
  if (!pService) {
    Serial.println("initBLE: ERROR createService() returned NULL");
    return;
  }

  // TX (notify) characteristic
  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  if (pTxCharacteristic) {
    pTxCharacteristic->addDescriptor(new BLE2902()); // client must enable notifications
    pTxCharacteristic->setValue("OK");
  } else {
    Serial.println("initBLE: WARNING pTxCharacteristic NULL");
  }

  // RX (write) characteristic (phone -> device)
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );
  if (!pRxCharacteristic) {
    Serial.println("initBLE: ERROR create RX characteristic");
  } else {
    pRxCharacteristic->setCallbacks(new ControllerBLECallbacks());
  }

  // start service
  pService->start();
  Serial.println("initBLE: service started");

  // advertise (both adv and scan response with name)
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x00);

  BLEAdvertisementData advData;
  advData.setName(BLE_DEVICE_NAME);
  advData.setFlags(0x06);
  String mfr = String("\x01\x02\x03\x04");
  advData.setManufacturerData(mfr);
  pAdvertising->setAdvertisementData(advData);

  BLEAdvertisementData scanResp;
  scanResp.setName(BLE_DEVICE_NAME);
  pAdvertising->setScanResponseData(scanResp);

  BLEDevice::startAdvertising();
  Serial.println("BLE advertising started");
}


void setup() {
  Serial.begin(115200); delay(200);
  initStorage(); prefs.begin("irrig", false);
  displayInitHeltec();
  loadSystemConfig();
    // load persisted manual mode & timeout
  manualMode = prefs.getBool(PREF_MANUAL_MODE, false);
  MANUAL_INACTIVITY_MS = prefs.getULong(PREF_MANUAL_TIMEOUT_MS, 0);
  if (manualMode) {
    Serial.println("BOOT: Starting in MANUAL mode (schedules disabled)");
    publishStatusIfAvailable("EVT|MODE|MANUAL|BOOT");
  }
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
  // try modem NTP at boot (best-effort)
  if (modemNtpSyncAndSetRTC()) {
    Serial.println("Boot: modem NTP OK");
  } else {
    Serial.println("Boot: modem NTP failed (will fall back as needed)");
  }
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
  // Do not trigger schedules while manual mode is active
  if (manualMode) { lastSchedulerCheck = millis(); }
  else {
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
}

  checkRtcDriftAndSync();
  //if (millis() - lastStatusPublish > statusPublishInterval) { publishStatusMsg(String("EVT|RUN|S=") + (scheduleRunning?String("1"):String("0"))); lastStatusPublish = millis(); } // need to fix ++++++++++++++++++++
  manualInactivityCheck();
  displayLoop();
  delay(20);
}

/* End of file */
