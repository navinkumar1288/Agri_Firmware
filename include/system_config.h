#pragma once
#include "storage.h"

struct SystemConfig {
  String mqttServer; int mqttPort; String mqttUser; String mqttPass; String adminPhones; String simApn; String sharedTok; String recoveryTok;
};

extern SystemConfig sysConfig;

void loadSystemConfig();
void saveSystemConfig();
String normalizePhone(const String &in);
std::vector<String> adminPhoneList();
bool isAdminNumber(const String &num);
