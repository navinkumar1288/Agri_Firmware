#include "scheduler.h"
#include "modem.h"
#include "system_config.h"
#include "radio.h"
#include "storage.h"
#include <ctype.h>

extern bool pumpIsOn;

std::vector<SeqStep> seq;
int currentStepIndex = -1;
unsigned long stepStartMillis = 0;
bool scheduleLoaded = false;
bool scheduleRunning = false;
unsigned long lastProgressSave = 0;
unsigned long lastStatusPublish = 0;
unsigned long statusPublishInterval = 15 * 1000;

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

String normalizePhoneLocal(const String &in) {
  String s = in; s.trim(); s.replace(" ", "");
  if (s.length() > 0 && s.charAt(0) == '0') s = s.substring(1);
  if (s.length() == 10 && s.charAt(0) != '+') s = String("+91") + s;
  return s;
}

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
  StaticJsonDocument<4096> scheduleDoc;
  DeserializationError err = deserializeJson(scheduleDoc, json);
  if (err) { dbg("JSON parse err"); return false; }
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
  dbg("Loaded schedule id=" + currentScheduleId + " seq size=" + String(seq.size()));
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

bool parseTimeHHMM(const String &t, int &hour, int &minute) {
  hour = 0; minute = 0; int res = sscanf(t.c_str(), "%d:%d", &hour, &minute); return res==2;
}
time_t computeNextRunEpoch(const Schedule &s, time_t now) {
  if (!s.enabled) return 0;
  if (s.rec == 'O') return s.start_epoch;
  // For daily/weekly, simplistic: schedule for today at start_time or next day
  if (s.timeStr.length()) {
    int h=0,m=0; if (parseTimeHHMM(s.timeStr, h, m)) {
      struct tm tmnow; localtime_r(&now, &tmnow);
      tmnow.tm_hour = h; tmnow.tm_min = m; tmnow.tm_sec = 0;
      time_t candidate = mktime(&tmnow);
      if (s.rec == 'D') {
        if (candidate <= now) candidate += 24*3600;
        return candidate;
      } else if (s.rec == 'W') {
        // find next weekday in mask
        for (int d=0; d<8; ++d) {
          int weekday = (localtime(&now)->tm_wday + d) % 7;
          if (s.weekday_mask & (1<<weekday)) {
            struct tm tm2 = tmnow;
            tm2.tm_mday += d;
            time_t cand2 = mktime(&tm2);
            if (cand2 > now) return cand2;
          }
        }
      }
    }
  }
  return 0;
}

void processIncomingScheduleString(const String &payload) {
  String trimmed = payload; trimmed.trim(); if (trimmed.length()==0) return;
  String src = extractSrc(trimmed);
  String fromNumber = extractKeyVal(trimmed, "_FROM");
  Serial.printf("Processing incoming payload from %s : %s\n", src.c_str(), trimmed.c_str());

  if (!verifyTokenForSrc(trimmed, fromNumber)) { publishStatusMsg(String("ERR|AUTH_FAIL|SRC=") + src); Serial.println("Auth failed"); return; }

  if (trimmed.startsWith("{") || trimmed.startsWith("[")) {
    if (validateAndLoadScheduleFromJson(trimmed)) broadcastStatus(String("EVT|SCH|SAVED|SRC=") + src);
    else publishStatusMsg("ERR|SCH|JSON_INVALID");
    return;
  }

  if (trimmed.indexOf("SCH|") >= 0) {
    if (saveCompactScheduleToMultipleFilesAndLoad(trimmed)) broadcastStatus(String("EVT|SCH|SAVED|S=") + currentScheduleId + String("|SRC=") + src);
    else publishStatusMsg("ERR|SCH|INVALID");
    return;
  }

  if (trimmed.indexOf("CFG|") >= 0) {
    String body = trimmed; int p = body.indexOf("CFG|"); if (p>=0) body = body.substring(p+4);
    if (processSystemConfigSms(body, fromNumber)) broadcastStatus(String("EVT|CFG|OK|SRC=") + src);
    else publishStatusMsg("ERR|CFG|INVALID");
    return;
  }

  if (trimmed.startsWith("{") && trimmed.indexOf("MS")>=0) {
    if (processSystemConfigJson(trimmed)) broadcastStatus(String("EVT|CFG|OK|SRC=") + src);
    else publishStatusMsg("ERR|CFG|INVALID");
    return;
  }

  Serial.println("Payload not recognized: " + trimmed);
  publishStatusMsg(String("ERR|UNKNOWN|SRC=") + src);
}

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
  if (!allowed && rec.length() && rec == sysConfig.recoveryTok) { allowed = true; Serial.println("Recovery token used by " + sender); }
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
      } else if (key == "SA") sysConfig.simApn = val;
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

void setPump(bool on) {
  pinMode(PUMP_PIN, OUTPUT);
  if (PUMP_ACTIVE_HIGH) digitalWrite(PUMP_PIN, on ? HIGH : LOW);
  else               digitalWrite(PUMP_PIN, on ? LOW  : HIGH);
  pumpIsOn = on;
  Serial.printf("Pump %s\n", on ? "ON" : "OFF");
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
    if (radioSendAndWaitAck(String("CMD|OPEN|N=")+String(node)+String(",S=")+currentScheduleId+String(",I=")+String(i)+String(",T=")+String(dur), node, i, LORA_ACK_TIMEOUT_MS)) { startIndex = (int)i; break; }
  }
  if (startIndex < 0) { publishStatusMsg("ERR|no_start_node_opened"); return; }
  for (size_t i=0;i<seq.size();++i) {
    if ((int)i == startIndex) continue;
    radioSendAndWaitAck(String("CMD|CLOSE|N=")+String(seq[i].node_id)+String(",S=")+currentScheduleId+String(",I=")+String(i), seq[i].node_id, i, LORA_ACK_TIMEOUT_MS);
  }
  setPump(true); delay(pumpOnBeforeMs);
  scheduleRunning = true; currentStepIndex = startIndex; stepStartMillis = millis();
  prefs.putInt("active_index", currentStepIndex);
  publishStatusMsg(String("EVT|START|S=") + currentScheduleId);
}

void stopSchedule() {
  if (currentStepIndex >=0 && currentStepIndex < (int)seq.size()) radioSendAndWaitAck(String("CMD|CLOSE|N=")+String(seq[currentStepIndex].node_id)+String(",S=")+currentScheduleId+String(",I=")+String(currentStepIndex), seq[currentStepIndex].node_id, currentStepIndex, LORA_ACK_TIMEOUT_MS);
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
      if (radioSendAndWaitAck(String("CMD|OPEN|N=")+String(seq[cand].node_id)+String(",S=")+currentScheduleId+String(",I=")+String(cand)+String(",T=")+String(seq[cand].duration_ms), seq[cand].node_id, cand, LORA_ACK_TIMEOUT_MS)) { nextIdx = cand; break; }
    }
    radioSendAndWaitAck(String("CMD|CLOSE|N=")+String(step.node_id)+String(",S=")+currentScheduleId+String(",I=")+String(currentStepIndex), step.node_id, currentStepIndex, LORA_ACK_TIMEOUT_MS);
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

void periodicTasks() {
  unsigned long nowMs = millis();
  static unsigned long lastSchedulerCheck = 0;
  if (nowMs - lastSchedulerCheck > 5000) {
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
    lastSchedulerCheck = nowMs;
  }

  checkRtcDriftAndSync();

  if (millis() - lastStatusPublish > statusPublishInterval) { publishStatusMsg(String("EVT|RUN|S=") + (scheduleRunning?String("1"):String("0"))); lastStatusPublish = millis(); }
}
