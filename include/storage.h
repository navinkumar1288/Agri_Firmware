#pragma once

struct SeqStep { int node_id; uint32_t duration_ms; };
struct Schedule {
  String id; char rec; time_t start_epoch; String timeStr; uint8_t weekday_mask;
  std::vector<SeqStep> seq; uint32_t pump_on_before_ms; uint32_t pump_off_after_ms;
  bool enabled; time_t next_run_epoch; uint32_t ts;
};

bool initStorage();
bool saveStringFile(const String &path, const String &content);
String loadStringFile(const String &path);

extern std::vector<Schedule> schedules;

bool saveScheduleFile(const Schedule &s);
Schedule scheduleFromJsonString(const String &json);
void loadAllSchedulesFromFS();
