#pragma once

bool initStorage();
bool saveStringFile(const String &path, const String &content);
String loadStringFile(const String &path);


bool saveScheduleFile(const Schedule &s);
Schedule scheduleFromJsonString(const String &json);
void loadAllSchedulesFromFS();
