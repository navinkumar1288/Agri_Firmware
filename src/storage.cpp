#include "storage.h"

std::vector<Schedule> schedules;

bool initStorage() {
  if (!LittleFS.begin(true)) { Serial.println("LittleFS mount failed"); return false; }
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

bool saveScheduleFile(const Schedule &s) {
  DynamicJsonDocument d(4096);
  d["schedule_id"] = s.id; d["recurrence"] = (s.rec=='D'?"daily":(s.rec=='W'?"weekly":"onetime"));
  d["start_time"] = s.timeStr; d["start_epoch"] = (long long)s.start_epoch;
  d["pump_on_before_ms"] = s.pump_on_before_ms; d["pump_off_after_ms"] = s.pump_off_after_ms; d["ts"] = s.ts;
  JsonArray arr = d.createNestedArray("sequence");
  for (auto &st : s.seq) { JsonObject so = arr.createNestedObject(); so["node_id"]=st.node_id; so["duration_ms"]=st.duration_ms; }
  String out; serializeJson(d, out);
  String path = String("/schedules/") + s.id + String(".json");
  return saveStringFile(path, out);
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
