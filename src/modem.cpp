#include "modem.h"
#include "config.h"
#include "system_config.h"
#include "storage.h"

HardwareSerial ModemSerial(2);
String modemLineBuffer = "";
unsigned long lastModemActivity = 0;
unsigned long lastMqttURCTime = 0;
bool mqttAvailable = true;
bool ENABLE_SMS_BROADCAST = false;

void publishStatusMsg(const String &msg) {
  String out = msg; Serial.println("PublishStatus: " + out);
  if (mqttAvailable) { modemPublish(MQTT_TOPIC_STATUS, out); }
  if (ENABLE_SMS_BROADCAST) {
    ModemSerial.print("AT+CMGF=1\r\n"); delay(50);
    ModemSerial.print(String("AT+CMGS=\"") + sysConfig.adminPhones + String("\"\r\n")); delay(200);
    ModemSerial.print(out); ModemSerial.write(0x1A); delay(100);
  }
  radioSend(String("STAT|") + out);
}
void broadcastStatus(const String &msg) { publishStatusMsg(msg); }

String sendAT(const String &cmd, unsigned long timeoutMs) {
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
  sendAT(String("AT+QMTSUB=0,1,\"") + MQTT_TOPIC_STATUS + String("\",1"), 5000);
  dbg("Modem MQTT setup attempted");
  return true;
}

bool modemPublish(const char* topic, const String &payload) {
  String esc = payload; esc.replace("\"", "\\\"");
  String cmd = String("AT+QMTPUB=0,0,0,1,\"") + String(topic) + String("\",\"") + esc + String("\"");
  String resp = sendAT(cmd, 6000);
  dbg(String("MQTT PUB resp: ") + resp);
  bool ok = resp.indexOf("OK") >= 0;
  if (!ok) {
    // fallback: sms alert to admins (comma-separated)
    sendSms(sysConfig.adminPhones, String("MQTTPUB FAIL:") + String(topic));
  }
  return ok;
}

bool sendSms(const String &to, const String &text) {
  if (to.length() == 0) return false;
  dbg("Sending SMS to " + to + ": " + text);
  sendAT("AT+CMGF=1",200);
  ModemSerial.print(String("AT+CMGS=\"") + to + String("\"\r"));
  delay(200);
  ModemSerial.print(text);
  ModemSerial.write((uint8_t)26);
  unsigned long start = millis(); String resp="";
  while (millis() - start < 8000) {
    while (ModemSerial.available()) resp += (char)ModemSerial.read();
    if (resp.indexOf("+CMGS:") >= 0 || resp.indexOf("OK") >= 0) break;
    delay(50);
  }
  dbg("SMS resp: " + resp);
  return resp.indexOf("OK") >= 0 || resp.indexOf("+CMGS:") >= 0;
}

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
