#pragma once

void radioInit();
void handleLoRaIncoming();
void radioSend(const String &payload);
bool radioSendAndWaitAck(const String &payload, uint32_t wantNode, uint32_t wantSeqIndex, uint32_t timeout_ms);
void radioSendAck(const String &toPayload);
extern void setModeManual();
extern void setModeAuto();
extern void stopSchedule();
extern void setPump(bool on);
