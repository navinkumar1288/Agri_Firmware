#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <RTClib.h>
#include "heltec.h"
#include "LoRaWan_APP.h"

// ---- Pins ----
#define MODEM_RX 45
#define MODEM_TX 46
#define MODEM_BAUD 115200
#define RTC_SDA 41
#define RTC_SCL 42
#define PUMP_PIN 25
#define PUMP_ACTIVE_HIGH true

// ---- WiFi & MQTT ----
#define WIFI_SSID "sekarfarm"
#define WIFI_PASS "welcome123"
#define DEFAULT_MQTT_SERVER "39aff691b9b5421ab98adc2addedbd83.s1.eu.hivemq.cloud"
#define DEFAULT_MQTT_PORT 8883
#define DEFAULT_MQTT_USER "navin"
#define DEFAULT_MQTT_PASS "HaiNavin33"
#define DEFAULT_SIM_APN "airtelgprs.com"

#define MQTT_TOPIC_SCHEDULE "irrigation/site01/schedule/set"
#define MQTT_TOPIC_CONFIG   "irrigation/site01/config/system/set"
#define MQTT_TOPIC_STATUS   "irrigation/site01/status"

// ---- LoRa ----
#define RF_FREQUENCY             865000000L
#define TX_POWER_DBM             14
#define LORA_BANDWIDTH           0
#define LORA_SPREADING_FACTOR    7
#define LORA_CODINGRATE          1
#define LORA_PREAMBLE_LENGTH     8
#define LORA_SYMBOL_TIMEOUT      0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON     false

// ---- BLE ----
#define BLE_DEVICE_NAME "IrrigCtrl"
#define BLE_SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_CHAR_UUID    "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// ---- Timings ----
#define LORA_ACK_TIMEOUT_MS 3000
#define LORA_MAX_RETRIES 3
#define SAVE_PROGRESS_INTERVAL_MS 10000
#define PUMP_ON_LEAD_DEFAULT_MS 2000
#define PUMP_OFF_DELAY_DEFAULT_MS 5000
#define LAST_CLOSE_DELAY_MS_DEFAULT 60000
#define WIFI_CONNECT_TIMEOUT_MS 20000
#define NTP_TIMEOUT_MS 20000
#define DEFAULT_RECOV_TOK "FARMTOK123"

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;
const int daylightOffset_sec = 0;

std::vector<SeqStep> seq;
int currentStepIndex = -1;
unsigned long stepStartMillis = 0;
bool scheduleLoaded = false;
bool scheduleRunning = false;
unsigned long lastProgressSave = 0;
unsigned long lastStatusPublish = 0;
unsigned long statusPublishInterval = 15 * 1000;
String currentScheduleId = "";
