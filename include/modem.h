#pragma once

void modemInit();
String sendAT(const String &cmd, unsigned long timeoutMs = 2000);
bool modemConfigureAndConnectMQTT();
bool modemPublish(const char* topic, const String &payload);
String modemReadSMSByIndex(int index, String &outSender);
void modemBackgroundRead();
bool sendSms(const String &to, const String &text);
