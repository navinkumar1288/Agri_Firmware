#include "ble_mod.h"
#include "utils.h"
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "config.h"

class ControllerBLECallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    auto v = pChar->getValue();
    String payload = String(v.c_str());
    payload.trim();
    if (payload.length() == 0) return;
    if (payload.indexOf("SRC=") < 0) payload += String(",SRC=BT");
    enqueueIncoming(payload);
    publishStatusMsg(String("EVT|INQ|ENQ|SRC=BT"));
  }
};

void initBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(BLE_CHAR_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
  pCharacteristic->setValue("OK"); pCharacteristic->setCallbacks(new ControllerBLECallbacks());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); pAdvertising->addServiceUUID(BLE_SERVICE_UUID); pAdvertising->start();
  Serial.println("BLE started");
}
