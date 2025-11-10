/*
  Node Controller - Heltec V3 (ESP32 + Radio driver + OLED)
  - Uses Heltec Radio driver (LoRaWan_APP.h) same pattern as Main Controller
  - Supports up to 4 valves, each up to 2 soil sensors grouped
  - Reports battery %, battery voltage, solar voltage, optional current
  - Handles CMD|MID=...|OPEN/CLOSE/STATUS and replies ACK|MID=...|...|OK|...
*/

#include <Arduino.h>
#include <Preferences.h>
#include "heltec.h"
#include "HT_SSD1306Wire.h"          // Heltec SSD1306 driver
#include <vector>
#include "LoRaWan_APP.h"   // Heltec radio driver (Radio.Init, Radio.Send, RadioEvents)
#include <Wire.h>

// ---------------- Display (Heltec) ----------------
// Use Heltec constructor that matches the installed HT_SSD1306Wire.h
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// -------------------- CONFIG / pins / LoRa params --------------------
// LoRa SPI pins (kept for clarity; Radio driver will use board definitions)
#define LORA_CS   18
#define LORA_RST  14
#define LORA_DIO0 26

// LoRa params used below (copied from main controller to match radio config)
#define RF_FREQUENCY         865000000 // Hz
#define TX_OUTPUT_POWER      5         // dBm
#define LORA_BANDWIDTH       0         // [0:125kHz,1:250kHz,2:500kHz]
#define LORA_SPREADING_FACTOR 7        // [SF7..SF12]
#define LORA_CODINGRATE      1         // [1:4/5,2:4/6,3:4/7,4:4/8]
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT  0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define BUFFER_SIZE 512

// Node config
#define DEFAULT_NODE_ID 2

// Up to 4 valves - set to -1 to disable a valve
#define VALVE_COUNT 4
int VALVE_PINS[VALVE_COUNT] = {41, 42, 45, 46};
bool VALVE_ACTIVE_HIGH[VALVE_COUNT] = {true, true, true, true};
bool valveOpen[VALVE_COUNT];
unsigned long valveOpenUntilMs[VALVE_COUNT];

// Soil sensors: per-valve up to 2 sensors (-1 = unused)
const int SOIL_SENSOR_PINS[4][2] = {
  {2, 3},
  {4, -1},
  {5, 6},
  {-1, -1}
};

// ADC / battery / solar pins & calibration
#define BATTERY_ADC_PIN   39
#define SOLAR_ADC_PIN     36
#define SOLAR_CURRENT_PIN -1

const float ADC_REF_VOLTAGE = 3.30f;
const int ADC_MAX = 4095;
const float BAT_DIVIDER_RATIO = 3.0f;
const float SOL_DIVIDER_RATIO = 4.0f;
const float BATTERY_MAX_VOLT = 4.20f;
const float BATTERY_MIN_VOLT = 3.30f;

const int SOIL_DRY_RAW = 3400;
const int SOIL_WET_RAW = 1600;

// Button
#define BUTTON_PIN 0
#define DEBOUNCE_MS 50

// Display refresh
const unsigned long DISPLAY_REFRESH_MS = 1000;

// -------------------- GLOBALS --------------------
Preferences prefs;
int NODE_ID = DEFAULT_NODE_ID;

String lastSchedId = "";
int lastSeqIndex = -1;
uint32_t lastCmdMid = 0;

volatile bool buttonPressed = false;
unsigned long lastButtonMs = 0;

static String disp_error = "";
static unsigned long lastDisplayMs = 0;
static unsigned long lastScrollMs = 0;
static int error_scroll_pos = 0;

// -------------------- Radio buffers & events --------------------
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents; // defined in LoRaWan_APP.h

// Forward declarations
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void handleRadioPayload(const char *payload, uint16_t size);

// -------------------- UTILITIES --------------------
String safeField(const String &s) {
  String out = s;
  out.replace("\n", "");
  out.replace("\r", "");
  out.replace(",", "_");
  return out;
}

void setValveState(int vidx, bool on) {
  if (vidx < 0 || vidx >= VALVE_COUNT) return;
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
float adcToVoltage(int raw) { return ((float)raw / (float)ADC_MAX) * ADC_REF_VOLTAGE; }
float readBatteryVoltage() {
  if (BATTERY_ADC_PIN < 0) return 0.0f;
  int raw = readAdcRaw(BATTERY_ADC_PIN); float v_adc = adcToVoltage(raw); return v_adc * BAT_DIVIDER_RATIO;
}
float batteryPctFromVoltage(float v) {
  if (v >= BATTERY_MAX_VOLT) return 100.0f;
  if (v <= BATTERY_MIN_VOLT) return 0.0f;
  return ((v - BATTERY_MIN_VOLT) / (BATTERY_MAX_VOLT - BATTERY_MIN_VOLT)) * 100.0f;
}
float readSolarVoltage() {
  if (SOLAR_ADC_PIN < 0) return 0.0f;
  int raw = readAdcRaw(SOLAR_ADC_PIN); float v_adc = adcToVoltage(raw); return v_adc * SOL_DIVIDER_RATIO;
}
float readSolarCurrent_mA() {
  if (SOLAR_CURRENT_PIN < 0) return -1.0f;
  int raw = readAdcRaw(SOLAR_CURRENT_PIN); float v_adc = adcToVoltage(raw);
  float v_mV = v_adc * 1000.0f; float vzero_mV = (ADC_REF_VOLTAGE / 2.0f) * 1000.0f;
  const float sensitivity_mV_per_A = 185.0f;
  float iA = (v_mV - vzero_mV) / sensitivity_mV_per_A;
  return iA * 1000.0f;
}
int moisturePercentFromRaw(int raw, int dryRaw = SOIL_DRY_RAW, int wetRaw = SOIL_WET_RAW) {
  if (raw <= 0) return 0;
  if (raw >= dryRaw) return 0;
  if (raw <= wetRaw) return 100;
  float pct = (float)(dryRaw - raw) / (float)(dryRaw - wetRaw) * 100.0f;
  if (pct < 0) pct = 0; if (pct > 100) pct = 100;
  return (int)round(pct);
}

// -------------------- Telemetry builder --------------------
String buildTelemetryExtra() {
  String extra = "";
  for (int i=0;i<VALVE_COUNT;i++) {
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
  // send via Radio driver
  snprintf(txpacket, BUFFER_SIZE, "%s", msg.c_str());
  Radio.Send((uint8_t *)txpacket, strlen(txpacket));
  Serial.println(String("[Radio TX] ") + msg);
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
  if (parts.size() < VALVE_COUNT) return false;
  String midPart = parts[1];
  if (!midPart.startsWith("MID=")) return false;
  outMid = (uint32_t)midPart.substring(VALVE_COUNT).toInt();
  outType = parts[2];
  String kv = parts[3];
  for (size_t i=VALVE_COUNT;i<parts.size();++i) kv += String("|") + parts[i];
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
      else if (k == "T") { unsigned long t = (unsigned long)v.toInt(); if (t>0 && t<=86400 && v.length()<=VALVE_COUNT) t = t*1000UL; outT = t; }
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
  if (s == "ALL") { for (int i=0;i<VALVE_COUNT;i++) if (VALVE_PINS[i] >= 0) res.push_back(i); return res; }
  int pos=0;
  while (pos < (int)s.length()) {
    int comma = s.indexOf(',', pos);
    String token = (comma == -1) ? s.substring(pos) : s.substring(pos, comma);
    token.trim();
    int dash = token.indexOf('-');
    if (dash == -1) {
      int v = token.toInt(); if (v>=1 && v<=VALVE_COUNT && VALVE_PINS[v-1]>=0) res.push_back(v-1);
    } else {
      int a = token.substring(0,dash).toInt(); int b = token.substring(dash+1).toInt();
      if (a<1) a=1; if (b>VALVE_COUNT) b=VALVE_COUNT;
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

// -------------------- RADIO: OnTx/OnRx handlers --------------------
void OnTxDone(void) {
  Serial.println("[Radio] TX done");
  // return to receive mode
  Radio.Rx(0);
}
void OnTxTimeout(void) {
  Serial.println("[Radio] TX timeout");
  Radio.Rx(0);
}
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  // copy into rxpacket and null-terminate
  if (size >= (int)sizeof(rxpacket)) size = sizeof(rxpacket)-1;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Serial.printf("[Radio] RX %d bytes RSSI=%d SNR=%d => %s\n", size, rssi, snr, rxpacket);
  // Handle payload directly (process CMDs here)
  handleRadioPayload(rxpacket, size);
  // go back to RX
  Radio.Rx(0);
}

// send using Radio.Send (non-blocking)
void sendLoRaPacketRadio(const String &msg) {
  snprintf(txpacket, BUFFER_SIZE, "%s", msg.c_str());
  Radio.Send((uint8_t *)txpacket, strlen(txpacket));
  Serial.printf("[Radio TX] %s\n", txpacket);
}

// process radio payload (string) similarly to earlier LoRa.parsePacket() branch
void handleRadioPayload(const char *payload, uint16_t size) {
  String msg = String(payload);
  msg.trim();
  if (msg.length() == 0) return;
  Serial.println(String("[RADIO PAYLOAD] ") + msg);
  // If payload doesn't have SRC, optionally add (we don't need to for node)
  // Now parse as CMD and react
  uint32_t mid=0; String type; int n=-1; String sched=""; int idx=-1; uint32_t t_ms=0; String vraw="";
  if (parseCmd(msg, mid, type, n, sched, idx, t_ms, vraw)) {
    Serial.printf("Parsed CMD MID=%u TYPE=%s N=%d V=%s S=%s I=%d T=%lu\n", (unsigned)mid, type.c_str(), n, vraw.c_str(), sched.c_str(), idx, (unsigned long)t_ms);
    if (n == NODE_ID || n == -1) {
      lastCmdMid = mid; lastSchedId = sched; lastSeqIndex = idx;
      std::vector<int> targets = parseValveSelector(vraw);
      if (targets.size() == 0 && VALVE_PINS[0] >= 0) targets.push_back(0);
      if (type == "OPEN") {
        for (int vidx : targets) {
          setValveState(vidx, true);
          if (t_ms > 0) valveOpenUntilMs[vidx] = millis() + t_ms; else valveOpenUntilMs[vidx] = 0;
        }
        // reply ACK
        String vlist="";
        for (size_t i=0;i<targets.size();++i) { if (i) vlist += ","; vlist += String(targets[i]+1); }
        sendAck(mid, "OPEN", NODE_ID, sched, idx, String("VLIST=") + vlist);
      } else if (type == "CLOSE") {
        for (int vidx : targets) {
          setValveState(vidx, false);
          valveOpenUntilMs[vidx] = 0;
        }
        String vlist="";
        for (size_t i=0;i<targets.size();++i) { if (i) vlist += ","; vlist += String(targets[i]+1); }
        sendAck(mid, "CLOSE", NODE_ID, sched, idx, String("VLIST=") + vlist);
      } else if (type == "STATUS" || type == "DETAIL" || type == "INFO") {
        String tele = buildTelemetryExtra();
        sendAck(mid, "STATUS", NODE_ID, sched, idx, tele);
      } else {
        sendAck(mid, type, NODE_ID, sched, idx, "ERR_UNKNOWN");
      }
    } else {
      Serial.printf("CMD for node %d ignoring (this node=%d)\n", n, NODE_ID);
    }
  } else {
    Serial.println("Radio payload not recognized as CMD.");
  }
}

// -------------------- Display (Heltec) --------------------

void VextON(){ pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW); }
void VextOFF(){ pinMode(Vext, OUTPUT); digitalWrite(Vext, HIGH); }

// -------------------- Display (Heltec) --------------------
void displayInitHeltec() {
  VextON(); 
  delay(50);
  display.init();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Node Controller");
  display.drawString(0, 12, "Initializing...");
  display.display();
  delay(300);
  display.clear(); 
  display.display();
}

String formatTimeShort() {
  unsigned long s = millis() / 1000;
  int hh = (s / 3600) % 24;
  int mm = (s / 60) % 60;
  char buf[16]; snprintf(buf, sizeof(buf), "%02d:%02d", hh, mm);
  return String(buf);
}

void displayLoop() {
  unsigned long nowMs = millis();
  if (nowMs - lastDisplayMs < DISPLAY_REFRESH_MS) return;
  lastDisplayMs = nowMs;

  String timeLine = "T:" + formatTimeShort();
  String statusLine = "";
  for (int i=0;i<VALVE_COUNT;i++) if (VALVE_PINS[i] >= 0) statusLine += String("V") + String(i+1) + (valveOpen[i] ? "O " : "C ");
  if (statusLine.length() == 0) statusLine = "No valves";

  float battV = readBatteryVoltage();
  float battPct = batteryPctFromVoltage(battV);
  float sVolt = (SOLAR_ADC_PIN >= 0) ? readSolarVoltage() : 0.0f;
  String nodeLine = String("B:") + String((int)round(battPct)) + String("% ") + String(battV,2) + String("V S:") + String(sVolt,2) + String("V");

  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, "Node:" + String(NODE_ID));
  display.drawString(0, 12, timeLine);
  display.drawString(0, 26, statusLine);
  display.drawString(0, 40, nodeLine);

  if (disp_error.length() > 0) {
    const int maxChars = 20;
    String vis;
    if ((int)disp_error.length() <= maxChars) vis = disp_error;
    else {
      int L = disp_error.length();
      for (int i = 0; i < maxChars; ++i) vis += disp_error[(error_scroll_pos + i) % L];
      if (millis() - lastScrollMs > 300) { lastScrollMs = millis(); error_scroll_pos++; if (error_scroll_pos > (int)disp_error.length()) error_scroll_pos = 0; }
    }
    display.drawRect(0, 48, 128, 16);
    display.drawString(2, 52, vis);
  } else {
    display.drawString(0, 60, "Status OK");
  }
  display.display();
}

// -------------------- Button ISR --------------------
void IRAM_ATTR buttonISR() {
  unsigned long now = millis();
  if (now - lastButtonMs < DEBOUNCE_MS) return;
  lastButtonMs = now;
  buttonPressed = true;
}

// -------------------- LoRa init (Radio driver) --------------------
void loraInit() {
  // Mcu.begin macro/setup used by the Heltec radio stack - same as main controller
#ifdef HELTEC_BOARD
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
#else
  // If HELTEC_BOARD macro not available in this library version, just print startup
  Serial.println("Mcu.begin: HELTEC_BOARD macro not present; continuing (may still work).");
#endif

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

  // Start in receive mode
  Radio.Rx(0);

  Serial.println("Heltec Radio LoRa init OK (Node)");
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
    Serial.print("Valve index: "); Serial.println(i);
    if (VALVE_PINS[i] >= 0) {
      Serial.print("VALVE_PINS index: "); Serial.println(i);
      pinMode(VALVE_PINS[i], OUTPUT);
      if (VALVE_ACTIVE_HIGH[i]) {digitalWrite(VALVE_PINS[i], LOW);}
      else {digitalWrite(VALVE_PINS[i], HIGH);}
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

  // LoRa using radio driver
  loraInit();

  Serial.println("Node setup complete.");
}

void loop() {
  // Manual button toggles valve1 for quick test
  if (buttonPressed) {
    buttonPressed = false;
    if (VALVE_PINS[0] >= 0) {
      setValveState(0, !valveOpen[0]);
      valveOpenUntilMs[0] = 0;
      String extra = buildTelemetryExtra();
      // send STAT via Radio
      sendLoRaPacketRadio(String("STAT|N=") + String(NODE_ID) + String("|") + extra);
      // force display update
      lastDisplayMs = 0;
    }
  }

  // Auto-close per valve timers
  unsigned long now = millis();
  for (int i=0;i<VALVE_COUNT;i++) {
    //Serial.print("Valve index: "); Serial.println(i);
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
      sendLoRaPacketRadio(extra);
      // refresh display
      lastDisplayMs = 0;
    }
  }

  displayLoop();
  delay(10);
}
