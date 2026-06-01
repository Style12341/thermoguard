#pragma once

#include <Arduino.h>
#include <DallasTemperature.h>
#include "Payload.h"

class SensorDevice {
  DeviceAddress _addr;
  String       _sensorId;
  IPayload*    _payload;

  static String addrToStr(const DeviceAddress& a) {
    char b[17];
    snprintf(b, sizeof(b), "%02X%02X%02X%02X%02X%02X%02X%02X",
             a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7]);
    return String(b);
  }

public:
  SensorDevice(const DeviceAddress& addr, IPayload* p)
    : _payload(p) {
    memcpy(_addr, addr, sizeof(DeviceAddress));
    _sensorId = addrToStr(_addr);
  }

  const String& sensorId() const { return _sensorId; }

  String topic(const String& appId) const {
    return "thermoguard/direct/" + appId + "/" + _sensorId + "/data";
  }

  float temperature(DallasTemperature& s) const {
    return s.getTempC(_addr);
  }

  String payload(float temp, int rssi, bool tempOk = true) const {
    return _payload->toJson(temp, rssi, tempOk);
  }

  void setPayload(IPayload* p) { _payload = p; }

  static bool isValidReading(float t) {
    return t != DEVICE_DISCONNECTED_C && t < 85.0f;
  }
};
