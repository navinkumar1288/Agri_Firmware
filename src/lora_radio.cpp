#include "lora_radio.h"

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
      String msg=""; while (LoRa.available()) msg += (char)LoRa.read();
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
