#pragma once
#include "storage.h"

void processIncomingScheduleString(const String &payload);
void publishStatusMsg(const String &msg);
void broadcastStatus(const String &msg);
bool processSystemConfigSms(const String &smsBody, const String &fromNumber);
bool processSystemConfigJson(const String &payload);
bool saveCompactScheduleToMultipleFilesAndLoad(const String &compact);
bool validateAndLoadScheduleFromJson(const String &json);

void setPump(bool on);
void setModeManual();
void setModeAuto();
void stopSchedule();

void runScheduleLoop();
void startScheduleIfDue();

void periodicTasks();
