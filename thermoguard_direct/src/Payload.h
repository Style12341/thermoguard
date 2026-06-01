#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

class IPayload {
public:
  virtual ~IPayload() = default;
  virtual String version() const = 0;
  virtual String toJson(float temp, int rssi, bool tempOk = true) const = 0;
};

class PayloadV1 : public IPayload {
public:
  String version() const override { return "1.0"; }

  String toJson(float temp, int rssi, bool tempOk = true) const override {
    JsonDocument doc;
    doc["version"] = "1.0";
    doc["temp"]    = tempOk ? temp : -500.0f;
    doc["rssi"]    = rssi;
    if (!tempOk) doc["temp_ok"] = false;
    String out;
    serializeJson(doc, out);
    return out;
  }
};
