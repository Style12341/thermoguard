#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include "Debug.h"

#define MAX_SENSORS 8
#define PREFS_NAMESPACE "thermo"
#define PREFS_CONFIG_KEY "cfg"
#define AP_SSID_BASE "TG-Setup"
#define PIN_ONEWIRE 32
#define FIRMWARE_VERSION "2.0.0"

struct AppConfig
{
  char mqttBroker[64] = "";
  uint16_t mqttPort = 1883;
  char mqttUser[32] = "";
  char mqttPass[64] = "";
  char appId[32] = "";
  uint8_t sensorCount = 0;
  uint32_t readIntervalSec = 30;
};

extern AppConfig config;

inline String deviceIdFromMac()
{
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char buf[13];
  snprintf(buf, sizeof(buf), "%02X%02X%02X%02X%02X%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

inline bool loadConfig()
{
  Preferences prefs;
  if (!prefs.begin(PREFS_NAMESPACE, false))
    return false;
  if (!prefs.isKey(PREFS_CONFIG_KEY))
  {
    prefs.end();
    return false;
  }

  String json = prefs.getString(PREFS_CONFIG_KEY, "");
  prefs.end();
  if (json.isEmpty())
    return false;

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err)
  {
    D_printf("Config JSON parse error: %s\n", err.c_str());
    return false;
  }

  strncpy(config.mqttBroker, doc["mqtt_broker"] | "", 63);
  config.mqttPort = doc["mqtt_port"] | 1883;
  strncpy(config.mqttUser, doc["mqtt_user"] | "", 31);
  strncpy(config.mqttPass, doc["mqtt_pass"] | "", 63);
  strncpy(config.appId, doc["app_id"] | "", 31);
  config.sensorCount = doc["sensor_count"] | 0;
  config.readIntervalSec = doc["read_interval"] | 30;

  return true;
}

inline void saveConfig()
{
  Preferences prefs;
  prefs.begin(PREFS_NAMESPACE, false);

  JsonDocument doc;
  doc["mqtt_broker"]   = config.mqttBroker;
  doc["mqtt_port"] = config.mqttPort;
  doc["mqtt_user"] = config.mqttUser;
  doc["mqtt_pass"] = config.mqttPass;
  doc["app_id"] = config.appId;
  doc["sensor_count"] = config.sensorCount;
  doc["read_interval"] = config.readIntervalSec;

  String json;
  serializeJson(doc, json);
  prefs.putString(PREFS_CONFIG_KEY, json);
  prefs.end();

  D_println("Config saved");
  D_printf("  MQTT:    %s:%u\n", config.mqttBroker, config.mqttPort);
  D_printf("  User:    %s\n", config.mqttUser);
  D_printf("  App ID:  %s\n", config.appId);
  D_printf("  Sensors: %u\n", config.sensorCount);
  D_printf("  Interval: %us\n", config.readIntervalSec);
}

inline void clearConfig()
{
  Preferences prefs;
  prefs.begin(PREFS_NAMESPACE, false);
  prefs.clear();
  prefs.end();
  D_println("Config cleared");
}
