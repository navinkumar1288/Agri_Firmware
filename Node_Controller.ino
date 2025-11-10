/*
  Node_Controller.ino
  Heltec V3 (ESP32 / ESP32-S3)
  - Uses LoRa.h (simple) to avoid Heltec Radio-driver compatibility issues
  - Up to 4 valves (each with up to 2 soil moisture sensors grouped)
  - OLED display replaces per-valve LEDs
  - Reports battery %, battery voltage, solar voltage, optional solar current
  - Responds to CMD|MID=..|OPEN/CLOSE/STATUS|... and sends ACK|MID=..|...|OK|...
  - Auto-close support T=ms in OPEN command
*/

#include <Arduino.h>
#include <Preferences.h>
//#include <LoRa.h>
#include "heltec.h"
#include <vector>

// -------------------- CONFIG / pins / calibration --------------------
// LoRa SPI pins (Heltec board defaults)
#define LORA_CS   18
#define LORA_RST  14
#define LORA_DIO0 26
const long LORA_FREQ = 865000000L; // 865 MHz (India) - set as long

#define DEFAULT_NODE_ID 2

// Up to 4 valves - set to -1 to disable a valve
const int VALVE_PINS[4] = {27, 12, 13, 25};
const bool VALVE_ACTIVE_HIGH[4] = {true, true, true, true};

// Soil sensors grouped by valve (each valve up to 2 sensors)
const int SOIL_SENSOR_PINS[4][2] = {
  {32, 33}, // valve1 sensors
  {34, -1}, // valve2 single sensor
  {35, 36}, // valve3 sensors
  {-1, -1}  // valve4 none
};

// ADC pins for battery and solar
#define BATTERY_ADC_PIN   39   // ADC pin for battery divider
#define SOLAR_ADC_PIN     36   // ADC pin for solar divider (optional)
#define SOLAR_CURRENT_PIN -1   // ADC pin for solar current sensor (-1 to disable)

// ADC / voltage conversion (calibrate)
const float ADC_REF_VOLTAGE = 3.30f;
const int ADC_MAX = 4095; // 12-bit

// Divider ratios: Vbat = Vadc * BAT_DIVIDER_RATIO
const float BAT_DIVIDER_RATIO = 3.0f; // set to your resistor ratio
const float SOL_DIVIDER_RATIO = 4.0f; // set to your resistor ratio

// Battery % mapping
const float BATTERY_MAX_VOLT = 4.20f;
const float BATTERY_MIN_VOLT = 3.30f;

// Soil sensor calibration (raw ADC)
const int SOIL_DRY_RAW = 3400;
const int SOIL_WET_RAW = 1600;

// Button (manual toggle) pin
#define BUTTON_PIN 0
#define DEBOUNCE_MS 50

// Display refresh interval
const unsigned long DISPLAY_REFRESH_MS = 1000;

// Buffer size for LoRa RX processing
#define RXBUF_SZ 512

// -------------------- GLOBALS --------------------
Preferences prefs;
int NODE_ID = DEFAULT_NODE_ID;

bool valveOpen[4] = {false,false,false,false};
unsigned long valveOpenUntilMs[4] = {0,0,0,0};
String lastSchedId = "";
int lastSeqIndex = -1;
uint32_t lastCmdMid = 0;

volatile bool buttonPressed = false;
unsigned long lastButtonMs = 0;

static String disp_error = "";
static unsigned long lastDisplayMs = 0;
static unsigned long lastScrollMs = 0;
static int error_scroll_pos = 0;

// -------------------- UTILITIES --------------------
String safeField(const String &s) {
  String out = s;
  out.replace("\n", "");
  out.replace("\r", "");
  out.replace(",", "_");
  return out;
}

void setValveState(int vidx, bool on) {
  if (vidx < 0 || vidx >= 4) return;
  int pin = VALVE_PINS[vidx];
  if (pin < 0) return;
  pinMode(pin, OUTPUT);
  if (VALVE_ACTIVE_HIGH[vidx]) digitalWrite(pin, on ? HIGH : LOW);
  else digitalWrite(pin, on ? LOW : HIGH);
  valveOpen[vidx] = on;
  Serial.printf("Valve %d %s\n", vidx+1, on ? "OPEN" : "CLOSED");
}

// -------------------- ADC helpers --------------------
int readAdcRaw(int pin) {
  if (pin < 0) return 0;
  return analogRead(pin);
}
float adcToVoltage(int raw) {
  return ((float)raw / (float)ADC_MAX) * ADC_REF_VOLTAGE;
}
float readBatteryVoltage() {
  if (BATTERY_ADC_PIN < 0) return 0.0f;
  int raw = readAdcRaw(BATTERY_ADC_PIN);
  float v_adc = adcToVoltage(raw);
  return v_adc * BAT_DIVIDER_RATIO;
}
float batteryPctFromVoltage(float v) {
  if (v >= BATTERY_MAX_VOLT) return 100.0f;
  if (v <= BATTERY_MIN_VOLT) return 0.0f;
  return ((v - BATTERY_MIN_VOLT) / (BATTERY_MAX_VOLT - BATTERY_MIN_VOLT)) * 100.0f;
}
float readSolarVoltage() {
  if (SOLAR_ADC_PIN < 0) return 0.0f;
  int raw = readAdcRaw(SOLAR_ADC_PIN);
  float v_adc = adcToVoltage(raw);
  return v_adc * SOL_DIVIDER_RATIO;
}
float readSolarCurrent_mA() {
  if (SOLAR_CURRENT_PIN < 0) return -1.0f;
  int raw = readAdcRaw(SOLAR_CURRENT_PIN);
  float v_adc = adcToVoltage(raw);
  float v_mV = v_adc * 1000.0f;
  float vzero_mV = (ADC_REF_VOLTAGE / 2.0f) * 1000.0f;
  const float sensitivity_mV_per_A = 185.0f; // change per current sensor type
  float iA = (v_mV - vzero_mV) / sensitivity_mV_per_A;
  return iA * 1000.0f;
}
int moisturePercentFromRaw(int raw, int dryRaw = SOIL_DRY_RAW, int wetRaw = SOIL_WET_RAW) {
  if (raw <= 0) return 0;
  if (raw >= dryRaw) return 0;
  if (raw <= wetRaw) return 100;
  float pct = (float)(dryRaw - raw) / (float)(dryRaw - wetRaw) * 100.0f;
  if (pct < 0) pct = 0;
  if (pct > 100) pct = 100;
  return (int)round(pct);
}

// -------------------- Telemetry builder --------------------
String buildTelemetryExtra() {
  String extra = "";
  for (int i=0;i<4;i++) {
    if (VALVE_PINS[i] < 0) continue;
    extra += String("VALVE") + String(i+1) + "=" + (valveOpen[i] ? "OPEN" : "CLOSED");
    unsigned long vt = 0;
    if (valveOpen[i] && valveOpenUntilMs[i] > millis()) vt = valveOpenUntilMs[i] - millis();
    extra += String(",VT") + String(i+1) + "=" + String(vt);
    for (int s=0; s<2; ++s) {
      int pin = SOIL_SENSOR_PINS[i][s];
      if (pin < 0) continue;
      int raw = readAdcRaw(pin);
      int perc = moisturePercentFromRaw(raw);
      extra += String(",M") + String(i+1) + "_" + String(s+1) + "=" + String(perc);
    }
    extra += String(",");
  }
  if (extra.endsWith(",")) extra.remove(extra.length()-1);

  float battV = readBatteryVoltage();
  float battPct = batteryPctFromVoltage(battV);
  extra += String(",BATT=") + String((int)round(battPct)) + String(",BV=") + String(battV, 2);

  float sVolt = (SOLAR_ADC_PIN >= 0) ? readSolarVoltage() : 0.0f;
  extra += String(",SOLV=") + String(sVolt, 2);
  float sCur = (SOLAR_CURRENT_PIN >= 0) ? readSolarCurrent_mA() : -1.0f;
  if (sCur >= 0) extra += String(",SOLI=") + String((int)round(sCur));
  return extra;
}

// -------------------- ACK builder --------------------
void sendAck(uint32_t mid, const String &type, int node, const String &sched, int seqIndex, const String &extra = "") {
  String kv = String("N=") + String(node) + String(",S=") + safeField(sched) + String(",I=") + String(seqIndex);
  String msg = String("ACK|MID=") + String(mid) + String("|") + type + String("|") + kv + String("|OK");
  if (extra.length()) msg += String("|") + extra;
  // send via LoRa
  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();
  Serial.println("LoRa SENT: " + msg);
}

// -------------------- CMD parsing --------------------
// CMD|MID=123|OPEN|N=2,V=1,S=SC001,I=1,T=60000
bool parseCmd(const String &msg, uint32_t &outMid, String &outType, int &outN, String &outS, int &outI, uint32_t &outT, String &outVraw) {
  outMid = 0; outType=""; outN=-1; outS=""; outI=-1; outT=0; outVraw="";
  String s = msg; s.trim();
  if (!s.startsWith("CMD|")) return false;
  std::vector<String> parts;
  int start = 0;
  for (int i=0;i<(int)s.length();++i) if (s[i]=='|') { parts.push_back(s.substring(start,i)); start = i+1; }
  parts.push_back(s.substring(start));
  if (parts.size() < 4) return false;
  String midPart = parts[1];
  if (!midPart.startsWith("MID=")) return false;
  outMid = (uint32_t)midPart.substring(4).toInt();
  outType = parts[2];
  String kv = parts[3];
  for (size_t i=4;i<parts.size();++i) kv += String("|") + parts[i];
  int pos = 0;
  while (pos < (int)kv.length()) {
    int comma = kv.indexOf(',', pos);
    String token = (comma == -1) ? kv.substring(pos) : kv.substring(pos, comma);
    token.trim();
    int eq = token.indexOf('=');
    if (eq > 0) {
      String k = token.substring(0, eq); String v = token.substring(eq+1);
      k.trim(); v.trim();
      if (k == "N") outN = v.toInt();
      else if (k == "S") outS = v;
      else if (k == "I") outI = v.toInt();
      else if (k == "T") { unsigned long t = (unsigned long)v.toInt(); if (t>0 && t<=86400 && v.length()<=6) t = t*1000UL; outT = t; } // allow up to 86400s
      else if (k == "V") outVraw = v; // valve selector
    }
    if (comma == -1) break;
    pos = comma + 1;
  }
  return true;
}

std::vector<int> parseValveSelector(const String &vraw) {
  std::vector<int> res;
  if (vraw.length() == 0) return res;
  String s = vraw; s.trim(); s.toUpperCase();
  if (s == "ALL") { for (int i=0;i<4;i++) if (VALVE_PINS[i] >= 0) res.push_back(i); return res; }
  int pos=0;
  while (pos < (int)s.length()) {
    int comma = s.indexOf(',', pos);
    String token = (comma == -1) ? s.substring(pos) : s.substring(pos, comma);
    token.trim();
    int dash = token.indexOf('-');
    if (dash == -1) {
      int v = token.toInt(); if (v>=1 && v<=4 && VALVE_PINS[v-1]>=0) res.push_back(v-1);
    } else {
      int a = token.substring(0,dash).toInt(); int b = token.substring(dash+1).toInt();
      if (a<1) a=1; if (b>4) b=4;
      for (int k=a;k<=b;++k) if (VALVE_PINS[k-1]>=0) res.push_back(k-1);
    }
    if (comma == -1) break;
    pos = comma + 1;
  }
  // dedupe
  std::vector<int> uniq;
  for (int x: res) { bool f=false; for (int y: uniq) if (x==y) {f=true; break;} if(!f) uniq.push_back(x); }
  return uniq;
}

// -------------------- LoRa init (robust) --------------------
void loraInit() {
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  Serial.println("Initializing LoRa...");
  bool ok = false;
  // try with PABOOST if library supports second arg
  // Many Heltec LoRa libs provide LoRa.begin(long, bool). Try it first.
  #if defined(ARDUINO_ARCH_ESP32)
  // attempt with PABOOST; if that fails try without second arg
  if (!ok) {
    Serial.println("Trying LoRa.begin(freq, true) ...");
    // some LoRa versions accept two args, others don't; guard runtime by try/catch is not available,
    // so attempt and if returns 0/false, fallback.
    if (LoRa.begin(LORA_FREQ, true) == 1 || LoRa.begin(LORA_FREQ, true) == 0) {
      // Unfortunately LoRa.begin may return int or bool depending on lib; check parse by reading LoRa.bandwidth? no reliable way.
      // So check LoRa.parsePacket availability next, or rely on return truthiness.
    }
  }
  #endif

  // Preferred pattern: try LoRa.begin(freq, true) and fallback to LoRa.begin(freq)
  // Some LoRa libs return 1 for success, others return non-zero; so test by checking radio state after a short delay.
  bool started = false;

  // First try with PABOOST (if function exists it will run; if library doesn't accept 2 args it will fail to compile — but common Heltec libs do accept it)
  #ifdef LORA_H_HAS_SECOND_ARG // not a standard macro; but we attempt guarded calls below
  #endif

  // We'll attempt both calls but in a way that compiles with both libs:
  // 1) attempt LoRa.begin(freq, true) via a lambda wrapped in a compile-time trick:
  {
    // try begin(freq, true) in a way that compiles on libs that support it:
    #if defined(ESP32) || defined(ESP32S3) || defined(ARDUINO_ARCH_ESP32)
    // Many Heltec LoRa libs support two-arg signature.
    bool tryOk = false;
    // Use a small block to ignore warnings if not matching exact prototype (should compile on Heltec).
    if (!started) {
      Serial.println("Attempting LoRa.begin(LORA_FREQ, true) ...");
      // Some libs return int, some bool, but both convert to bool. We'll check return truthiness.
      if (LoRa.begin(LORA_FREQ, true)) {
        started = true; tryOk = true;
        Serial.println("LoRa.begin(LORA_FREQ, true) succeeded.");
      } else {
        Serial.println("LoRa.begin(LORA_FREQ, true) failed.");
      }
    }
    (void)tryOk;
    #endif
  }

  // 2) fallback: try single-arg begin
  if (!started) {
    Serial.println("Attempting LoRa.begin(LORA_FREQ) ...");
    if (LoRa.begin(LORA_FREQ)) {
      started = true;
      Serial.println("LoRa.begin(LORA_FREQ) succeeded.");
    } else {
      Serial.println("LoRa.begin(LORA_FREQ) failed.");
    }
  }

  if (!started) {
    Serial.println("LoRa init failed. Check wiring & library.");
  } else {
    Serial.println("LoRa init OK");
  }

  delay(50);
}

// -------------------- LoRa receive handling --------------------
void handleLoRaReceive() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;
  char buf[RXBUF_SZ];
  int idx = 0;
  while (LoRa.available() && idx < (int)sizeof(buf)-1) {
    int c = LoRa.read();
    if (c < 0) break;
    buf[idx++] = (char)c;
  }
  buf[idx] = '\0';
  String payload = String(buf);
  payload.trim();
  Serial.printf("[LoRa RX] %s\n", payload.c_str());
  if (payload.length() == 0) return;

  // parse & handle command (same logic used in Radio version)
  uint32_t mid=0; String type; int n=-1; String sched=""; int idxSeq=-1; uint32_t t_ms=0; String vraw="";
  if (parseCmd(payload, mid, type, n, sched, idxSeq, t_ms, vraw)) {
    Serial.printf("CMD MID=%u TYPE=%s N=%d V=%s S=%s I=%d T=%lu\n", (unsigned)mid, type.c_str(), n, vraw.c_str(), sched.c_str(), idxSeq, (unsigned long)t_ms);
    if (n == NODE_ID || n == -1) {
      lastCmdMid = mid; lastSchedId = sched; lastSeqIndex = idxSeq;
      std::vector<int> targets = parseValveSelector(vraw);
      if (targets.size() == 0 && VALVE_PINS[0] >= 0) targets.push_back(0);
      if (type == "OPEN") {
        for (int vidx : targets) {
          setValveState(vidx, true);
          if (t_ms > 0) valveOpenUntilMs[vidx] = millis() + t_ms; else valveOpenUntilMs[vidx] = 0;
        }
        String vlist="";
        for (size_t i=0;i<targets.size();++i) { if (i) vlist += ","; vlist += String(targets[i]+1); }
        sendAck(mid,"OPEN",NODE_ID,sched,idxSeq,String("VLIST=")+vlist);
      } else if (type == "CLOSE") {
        for (int vidx : targets) { setValveState(vidx, false); valveOpenUntilMs[vidx] = 0; }
        String vlist="";
        for (size_t i=0;i<targets.size();++i) { if (i) vlist += ","; vlist += String(targets[i]+1); }
        sendAck(mid,"CLOSE",NODE_ID,sched,idxSeq,String("VLIST=")+vlist);
      } else if (type == "STATUS" || type == "DETAIL" || type == "INFO") {
        String tele = buildTelemetryExtra();
        sendAck(mid,"STATUS",NODE_ID,sched,idxSeq,tele);
      } else {
        sendAck(mid,type,NODE_ID,sched,idxSeq,"ERR_UNKNOWN");
      }
    } else {
      Serial.printf("CMD for node %d ignoring (this=%d)\n", n, NODE_ID);
    }
  } else {
    Serial.println("LoRa payload not recognized as CMD.");
  }
}

// -------------------- OLED Display --------------------
void displayInitHeltec() {
  Heltec.begin(true /*DisplayEnable*/, false /*LoRaEnable*/, true /*SerialEnable*/);
  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, "Node Controller");
  Heltec.display->drawString(0, 12, "Initializing...");
  Heltec.display->display();
  delay(300);
  Heltec.display->clear(); Heltec.display->display();
}

String formatTimeShort() {
  unsigned long ms = millis() / 1000;
  int hh = (ms / 3600) % 24;
  int mm = (ms / 60) % 60;
  char buf[16]; snprintf(buf, sizeof(buf), "%02d:%02d", hh, mm);
  return String(buf);
}

void displayLoop() {
  unsigned long nowMs = millis();
  if (nowMs - lastDisplayMs < DISPLAY_REFRESH_MS) return;
  lastDisplayMs = nowMs;

  String timeLine = "T:" + formatTimeShort();
  String statusLine = "";
  for (int i=0;i<4;i++) if (VALVE_PINS[i] >= 0) statusLine += String("V") + String(i+1) + (valveOpen[i] ? "O " : "C ");
  if (statusLine.length() == 0) statusLine = "No valves";

  float battV = readBatteryVoltage();
  float battPct = batteryPctFromVoltage(battV);
  float sVolt = (SOLAR_ADC_PIN >= 0) ? readSolarVoltage() : 0.0f;
  String nodeLine = String("B:") + String((int)round(battPct)) + String("% ") + String(battV,2) + String("V S:") + String(sVolt,2) + String("V");

  Heltec.display->clear();
  Heltec.display->setFont(ArialMT_Plain_10);
  Heltec.display->drawString(0, 0, "Node:" + String(NODE_ID));
  Heltec.display->drawString(0, 12, timeLine);
  Heltec.display->drawString(0, 26, statusLine);
  Heltec.display->drawString(0, 40, nodeLine);

  if (disp_error.length() > 0) {
    const int maxChars = 20;
    String vis;
    if ((int)disp_error.length() <= maxChars) vis = disp_error;
    else {
      int L = disp_error.length();
      for (int i = 0; i < maxChars; ++i) vis += disp_error[(error_scroll_pos + i) % L];
      if (millis() - lastScrollMs > 300) { lastScrollMs = millis(); error_scroll_pos++; if (error_scroll_pos > (int)disp_error.length()) error_scroll_pos = 0; }
    }
    Heltec.display->drawRect(0, 48, 128, 16);
    Heltec.display->drawString(2, 52, vis);
  } else {
    Heltec.display->drawString(0, 60, "Status OK");
  }
  Heltec.display->display();
}

// -------------------- Button ISR --------------------
void IRAM_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - lastButtonMs < DEBOUNCE_MS) return;
  lastButtonMs = now;
  buttonPressed = true;
}

// -------------------- Setup / Loop --------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) delay(10);

  prefs.begin("nodecfg", false);
  NODE_ID = prefs.getInt("node_id", DEFAULT_NODE_ID);
  Serial.printf("Node ID = %d\n", NODE_ID);

  // initialize valve pins (closed)
  for (int i=0;i<4;i++) {
    if (VALVE_PINS[i] >= 0) {
      pinMode(VALVE_PINS[i], OUTPUT);
      if (VALVE_ACTIVE_HIGH[i]) digitalWrite(VALVE_PINS[i], LOW);
      else digitalWrite(VALVE_PINS[i], HIGH);
      valveOpen[i] = false;
      valveOpenUntilMs[i] = 0;
    }
  }

  analogReadResolution(12);

  // button
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);

  // display
  displayInitHeltec();

  // LoRa
  loraInit();

  Serial.println("Setup complete.");
}

void loop() {
  // manual button toggles valve1 for quick test
  if (buttonPressed) {
    buttonPressed = false;
    if (VALVE_PINS[0] >= 0) {
      setValveState(0, !valveOpen[0]);
      valveOpenUntilMs[0] = 0;
      String extra = buildTelemetryExtra();
      // send STAT
      LoRa.beginPacket();
      LoRa.print(String("STAT|N=") + String(NODE_ID) + String("|") + extra);
      LoRa.endPacket();
      lastDisplayMs = 0;
    }
  }

  // LoRa incoming handling
  handleLoRaReceive();

  // Auto-close per-valve timers
  unsigned long now = millis();
  for (int i=0;i<4;i++) {
    if (VALVE_PINS[i] >= 0 && valveOpen[i] && valveOpenUntilMs[i] > 0 && now >= valveOpenUntilMs[i]) {
      Serial.printf("Auto-close valve %d\n", i+1);
      setValveState(i, false);
      valveOpenUntilMs[i] = 0;
      // notify controller of auto-close (no MID)
      String extra = String("AUTO_CLOSED|N=") + String(NODE_ID) + String("|S=") + safeField(lastSchedId) + String(",I=") + String(lastSeqIndex);
      extra += String(",VALVE") + String(i+1) + "=CLOSED";
      float battV = readBatteryVoltage();
      extra += String(",BATT=") + String((int)round(batteryPctFromVoltage(battV))) + String(",BV=") + String(battV,2);
      for (int s=0;s<2;s++) {
        int pin = SOIL_SENSOR_PINS[i][s];
        if (pin>=0) {
          int raw = readAdcRaw(pin);
          extra += String(",M") + String(i+1) + "_" + String(s+1) + "=" + String(moisturePercentFromRaw(raw));
        }
      }
      LoRa.beginPacket();
      LoRa.print(extra);
      LoRa.endPacket();
      lastDisplayMs = 0;
    }
  }

  // Display update
  displayLoop();

  delay(10);
}
