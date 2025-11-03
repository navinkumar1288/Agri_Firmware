#pragma once
#include "utils.h"
#include <ArduinoJson.h>



bool initStorage();
bool saveStringFile(const String &path, const String &content);
String loadStringFile(const String &path);

extern std::vector<Schedule> schedules;

bool saveScheduleFile(const Schedule &s);
Schedule scheduleFromJsonString(const String &json);
void loadAllSchedulesFromFS();
