#include "radio.h"
#include "utils.h"

static RadioEvents_t RadioEvents;
volatile bool rxReceivedFlag = false;
static char rxPayload[512];
volatile int rxRssi = 0;
volatile int rxSnr = 0;

void OnTxDone(void) { dbg("Radio: TX done"); }
void OnTxTimeout(void) { dbg("Radio: TX timeout"); }
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  if (size >= (int)sizeof(rxPayload)) size = sizeof(rxPayload)-1;
  memcpy(rxPayload, payload, size);
  rxPayload[size] = 0;
  rxRssi = rssi; rxSnr = snr;
  rxReceivedFlag = true;
  dbg(String("Radio: RxDone payload: ") + String(rxPayload) + " rssi:" + String(rssi));
}
void OnRxTimeout(void) { dbg("Radio: Rx timeout"); }
void OnRxError(void) { dbg("Radio: Rx error"); }

void radioInit() {
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxError = OnRxError;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_POWER_DBM, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  dbg("Radio initialized");
}

static std::vector<String> splitPipe(const String &s) {
  std::vector<String> parts;
  int p = 0;
  for (int i = 0; i < (int)s.length(); ++i) if (s[i] == '|') { parts.push_back(s.substring(p, i)); p = i + 1; }
  parts.push_back(s.substring(p));
  return parts;
}

void radioSend(const String &payload) { Radio.Send((uint8_t*)payload.c_str(), payload.length()); }
void radioSendAck(const String &toPayload) {
  String ack = String("ACK|MAIN|") + toPayload + "|OK";
  dbg("Radio sending ACK: " + ack);
  Radio.Send((uint8_t*)ack.c_str(), ack.length());
}

bool radioSendAndWaitAck(const String &payload, uint32_t wantNode, uint32_t wantSeqIndex, uint32_t timeout_ms) {
  uint8_t tries = 0;
  while (tries < LORA_MAX_RETRIES) {
    dbg("Radio TX try " + String(tries + 1) + " -> " + payload);
    Radio.Send((uint8_t*)payload.c_str(), payload.length());
    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
      Radio.IrqProcess();
      if (rxReceivedFlag) {
        rxReceivedFlag = false;
        String r = String(rxPayload);
        dbg("Radio RX (waiting ack): " + r);
        auto parts = splitPipe(r);
        if (parts.size() >= 6 && parts[0] == "ACK") {
          int node = parts[2].toInt();
          int seqIdx = parts[4].toInt();
          String status = parts[5];
          if ((uint32_t)node == wantNode && (uint32_t)seqIdx == wantSeqIndex && status == "OK") {
            dbg("ACK matched for node " + String(node) + " seq " + String(seqIdx));
            return true;
          }
        }
      }
      delay(10);
    }
    tries++;
    dbg("No matching ACK received, retrying");
    delay(100);
  }
  dbg("radioSendAndWaitAck: ACK not received after retries");
  return false;
}

void handleLoRaIncoming() {
  if (!rxReceivedFlag) return;
  rxReceivedFlag = false;
  String msg = String(rxPayload);
  dbg("Processing incoming LoRa msg: " + msg);
  // split by '|'
  std::vector<String> parts;
  int p = 0;
  for (int i = 0; i < (int)msg.length(); ++i) {
    if (msg[i] == '|') { parts.push_back(msg.substring(p, i)); p = i+1; }
  }
  parts.push_back(msg.substring(p));
  if (parts.size() >= 2 && parts[0] == "CMD") {
    String cmd = parts[1]; cmd.toUpperCase();
    if (cmd == "SET" && parts.size() >= 4) {
      String target = parts[2]; target.toUpperCase();
      String val = parts[3]; val.toUpperCase();
      if (target == "PUMP") {
        if (val == "ON") { setPump(true); publishStatus("pump_remote_on"); }
        else if (val == "OFF") { setPump(false); publishStatus("pump_remote_off"); }
        radioSendAck(msg);
        return;
      }
    } else if (cmd == "MODE" && parts.size() >= 3) {
      String m = parts[2]; m.toUpperCase(); if (m == "MANUAL") setModeManual(); else setModeAuto(); radioSendAck(msg); return;
    } else if (cmd == "SCHEDULE" && parts.size() >= 3) {
      String act = parts[2]; act.toUpperCase(); if (act == "STOP") { stopSchedule(); radioSendAck(msg); return; }
    }
    dbg("Unknown CMD received: " + msg);
    radioSendAck(msg);
    return;
  }
  if (msg.startsWith("TR|")) {
    dbg("Telemetry request received: " + msg);
    String resp = String("T|MAIN|{\"ts\":\"") + nowISO8601() + String("\",\"pump\":\"") + (pumpIsOn ? "ON" : "OFF") + String("\"}");
    Radio.Send((uint8_t*)resp.c_str(), resp.length());
    return;
  }
  dbg("Unrecognized LoRa payload: " + msg);
}
