#include "system_config.h"

SystemConfig sysConfig;

String normalizePhone(const String &in) {
  String s = in; s.trim(); s.replace(" ", "");
  if (s.length() > 0 && s.charAt(0) == '0') s = s.substring(1);
  if (s.length() == 10 && s.charAt(0) != '+') s = String("+91") + s;
  return s;
}
std::vector<String> adminPhoneList() {
  std::vector<String> out;
  String s = sysConfig.adminPhones; s.trim();
  if (s.length() == 0) return out;
  int p = 0;
  while (p < (int)s.length()) {
    int c = s.indexOf(',', p);
    String part = (c==-1) ? s.substring(p) : s.substring(p, c);
    part.trim();
    if (part.length()) out.push_back(part);
    if (c == -1) break;
    p = c + 1;
  }
  return out;
}
bool isAdminNumber(const String &num) {
  String n = normalizePhone(num);
  auto list = adminPhoneList();
  for (auto &p : list) if (normalizePhone(p) == n) return true;
  return false;
}

void loadSystemConfig() {
  sysConfig.mqttServer = prefs.getString("mqtt_server", DEFAULT_MQTT_SERVER);
  sysConfig.mqttPort = prefs.getInt("mqtt_port", DEFAULT_MQTT_PORT);
  sysConfig.mqttUser = prefs.getString("mqtt_user", DEFAULT_MQTT_USER);
  sysConfig.mqttPass = prefs.getString("mqtt_pass", DEFAULT_MQTT_PASS);
  sysConfig.adminPhones = prefs.getString("admin_phones", "+919944272647");
  sysConfig.simApn = prefs.getString("sim_apn", DEFAULT_SIM_APN);
  sysConfig.sharedTok = prefs.getString("shared_tok", "MYTOK");
  sysConfig.recoveryTok = prefs.getString("recovery_tok", DEFAULT_RECOV_TOK);
  LAST_CLOSE_DELAY_MS = prefs.getULong("last_close_delay_ms", LAST_CLOSE_DELAY_MS_DEFAULT);
  DRIFT_THRESHOLD_S = prefs.getUInt("drift_s", DRIFT_THRESHOLD_S);
  uint32_t sync_h = prefs.getUInt("sync_h", (uint32_t)(SYNC_CHECK_INTERVAL_MS/3600000UL));
  SYNC_CHECK_INTERVAL_MS = (uint32_t)sync_h * 3600UL * 1000UL;
  Serial.println("Loaded system config.");
}
void saveSystemConfig() {
  prefs.putString("mqtt_server", sysConfig.mqttServer);
  prefs.putInt("mqtt_port", sysConfig.mqttPort);
  prefs.putString("mqtt_user", sysConfig.mqttUser);
  prefs.putString("mqtt_pass", sysConfig.mqttPass);
  prefs.putString("admin_phones", sysConfig.adminPhones);
  prefs.putString("sim_apn", sysConfig.simApn);
  prefs.putString("shared_tok", sysConfig.sharedTok);
  prefs.putString("recovery_tok", sysConfig.recoveryTok);
  prefs.putULong("last_close_delay_ms", LAST_CLOSE_DELAY_MS);
  prefs.putUInt("drift_s", DRIFT_THRESHOLD_S);
  prefs.putUInt("sync_h", (uint32_t)(SYNC_CHECK_INTERVAL_MS/3600000UL));
  Serial.println("Saved system config to prefs.");
}
