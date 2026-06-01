#include "Debug.h"
#include "Config.h"
#include "Payload.h"
#include "SensorDevice.h"

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ---- Globals ----
AppConfig         config;
AsyncWebServer    server(80);
WiFiClient        wifiClient;
PubSubClient      mqtt(wifiClient);
OneWire           oneWire(PIN_ONEWIRE);
DallasTemperature sensors(&oneWire);
WiFiManager       wm;

PayloadV1         payloadV1;
SensorDevice*     devices[MAX_SENSORS] = {};
uint8_t           deviceCount         = 0;

unsigned long     lastReadMs          = 0;
unsigned long     lastMqttAttemptMs   = 0;
String            deviceId            = "";

constexpr int     NUM_WM_PARAMS       = 7;
WiFiManagerParameter* wmParams[NUM_WM_PARAMS] = {};

// ---- MQTT Config Callback ----
void configCallback(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    D_printf("Config parse error: %s\n", err.c_str());
    return;
  }

  if (!doc.containsKey("send_interval")) return;

  long interval = doc["send_interval"];
  if (interval < 5 || interval > 3600) {
    D_printf("Config rejected: send_interval=%ld out of range (5-3600) from %s\n",
             interval, topic);
    return;
  }

  config.readIntervalSec = (uint32_t)interval;
  saveConfig();
  D_printf("Config applied: send_interval=%us from %s\n",
           config.readIntervalSec, topic);
}

// ---- Helpers ----
void discover(DeviceAddress* addrs, uint8_t& count) {
  sensors.begin();
  count = sensors.getDeviceCount();
  if (count > MAX_SENSORS) count = MAX_SENSORS;
  for (int i = 0; i < count; i++) {
    sensors.getAddress(addrs[i], i);
  }
}

bool mqttConnect() {
  if (mqtt.connected()) return true;

  String clientId    = "ThermoGuard-" + deviceId;
  String statusTopic = "thermoguard/direct/" + String(config.appId) + "/status";

  bool ok = strlen(config.mqttUser) > 0
    ? mqtt.connect(clientId.c_str(), config.mqttUser, config.mqttPass,
                   statusTopic.c_str(), 0, true, "{\"online\":false}")
    : mqtt.connect(clientId.c_str(),
                   statusTopic.c_str(), 0, true, "{\"online\":false}");

  if (ok) {
    mqtt.publish(statusTopic.c_str(), "{\"online\":true}", true);

    for (int i = 0; i < deviceCount; i++) {
      String topic = "thermoguard/direct/" + String(config.appId) + "/" +
                     devices[i]->sensorId() + "/config";
      mqtt.subscribe(topic.c_str());
      D_printf("Subscribed to config: %s\n", topic.c_str());
    }
    return true;
  }
  D_printf("MQTT rc=%d\n", mqtt.state());
  return false;
}

// ---- WiFiManager callbacks ----
bool shouldSaveConfig = false;

void onWmSave() {
  shouldSaveConfig = true;
}

void buildWmParams() {
  int i = 0;

  wmParams[i++] = new WiFiManagerParameter(
    "mqtt_broker", "MQTT Broker", config.mqttBroker, 64,
    "placeholder=\"mqtt.example.com\"");
  char ps[6]; snprintf(ps, sizeof(ps), "%u", config.mqttPort);
  wmParams[i++] = new WiFiManagerParameter(
    "mqtt_port", "MQTT Port", ps, 6, "placeholder=\"1883\"");
  wmParams[i++] = new WiFiManagerParameter(
    "mqtt_user", "MQTT User", config.mqttUser, 32,
    "placeholder=\"username\"");
  wmParams[i++] = new WiFiManagerParameter(
    "mqtt_pass", "MQTT Password", config.mqttPass, 64,
    "placeholder=\"password\" type=\"password\"");
  wmParams[i++] = new WiFiManagerParameter(
    "app_id", "App ID", config.appId, 32,
    "placeholder=\"greenhouse-1\"");

  char cnt[4]; snprintf(cnt, sizeof(cnt), "%u", config.sensorCount > 0 ? config.sensorCount : 1);
  wmParams[i++] = new WiFiManagerParameter(
    "sensor_count", "Sensor Count (1-8)", cnt, 3,
    "placeholder=\"1\" type=\"number\" min=\"1\" max=\"8\"");

  char intv[6]; snprintf(intv, sizeof(intv), "%u", config.readIntervalSec);
  wmParams[i++] = new WiFiManagerParameter(
    "read_interval", "Read Interval (sec)", intv, 6,
    "placeholder=\"30\" type=\"number\" min=\"5\"");

  for (int j = 0; j < NUM_WM_PARAMS; j++) wm.addParameter(wmParams[j]);
}

void readParamsFromWm() {
  int p = 0;
  strncpy(config.mqttBroker,  wmParams[p++]->getValue(), 63);
  config.mqttPort   = atoi(   wmParams[p++]->getValue());
  strncpy(config.mqttUser,    wmParams[p++]->getValue(), 31);
  strncpy(config.mqttPass,    wmParams[p++]->getValue(), 63);
  strncpy(config.appId,       wmParams[p++]->getValue(), 31);
  config.sensorCount = atoi(  wmParams[p++]->getValue());
  if (config.sensorCount > MAX_SENSORS) config.sensorCount = MAX_SENSORS;
  config.readIntervalSec = atoi(wmParams[p++]->getValue());
  if (config.readIntervalSec < 5) config.readIntervalSec = 5;
  saveConfig();
}

// ---- Setup ----
void setup() {
  D_SerialBegin(115200);
  delay(500);
  D_println();
  D_printf("ThermoGuard v%s\n", FIRMWARE_VERSION);

  deviceId = deviceIdFromMac();
  D_printf("Device ID: %s\n", deviceId.c_str());

  String apSsid = String(AP_SSID_BASE) + "-" + deviceId;

  loadConfig();

  pinMode(0, INPUT_PULLUP);
  if (digitalRead(0) == LOW) {
    unsigned long press = millis();
    while (digitalRead(0) == LOW) {
      if (millis() - press > 5000) {
        D_println("Factory reset: forgetting WiFi, keeping app params.");
        wm.resetSettings();
        config.sensorCount = 0;
        saveConfig();
        delay(500);
        ESP.restart();
      }
      delay(10);
    }
  }

  if (!config.sensorCount) {
    D_println("No config. Starting config portal...");
    shouldSaveConfig = false;
    buildWmParams();
    wm.setSaveConfigCallback(onWmSave);
    wm.setConfigPortalTimeout(300);
    wm.startConfigPortal(apSsid.c_str());

    if (shouldSaveConfig) {
      readParamsFromWm();
    }

    D_println("Restarting after config...");
    delay(2000);
    ESP.restart();
  }

  D_println("Config found, connecting WiFi...");
  wm.setConfigPortalTimeout(120);
  if (!wm.autoConnect(apSsid.c_str())) {
    D_println("WiFi failed, restarting...");
    delay(1000);
    ESP.restart();
  }
  D_printf("WiFi OK. IP: %s\n", WiFi.localIP().toString().c_str());

  ElegantOTA.onStart([]() { D_println("OTA Start"); });
  ElegantOTA.onEnd([](bool ok) { D_printf("OTA %s\n", ok ? "OK" : "FAIL"); });
  ElegantOTA.onProgress([](unsigned int p, unsigned int t) {
    D_printf("OTA: %u%%\r", (p * 100 / t));
  });
  ElegantOTA.begin(&server);
  server.begin();
  D_println("ElegantOTA ready");

  mqtt.setServer(config.mqttBroker, config.mqttPort);
  mqtt.setKeepAlive(60);
  mqtt.setCallback(configCallback);
  D_printf("MQTT: %s:%u  user=%s  appId=%s\n",
           config.mqttBroker, config.mqttPort,
           config.mqttUser, config.appId);

  DeviceAddress addrs[MAX_SENSORS];
  uint8_t found = 0;
  discover(addrs, found);
  D_printf("DS18B20: found %u, configured %u\n", found, config.sensorCount);

  for (int i = 0; i < found && i < config.sensorCount; i++) {
    devices[i] = new SensorDevice(addrs[i], &payloadV1);
    deviceCount++;
    D_printf("  [%d] %s\n", i, devices[i]->sensorId().c_str());
  }

  D_println("Setup complete.");
}

// ---- Loop ----
void loop() {
  ElegantOTA.loop();
  mqtt.loop();

  unsigned long now = millis();

  if (!mqtt.connected() && (now - lastMqttAttemptMs > 10000)) {
    lastMqttAttemptMs = now;
    mqttConnect();
  }

  if (mqtt.connected() && (now - lastReadMs > config.readIntervalSec * 1000UL)) {
    lastReadMs = now;

    sensors.requestTemperatures();
    int rssi = WiFi.RSSI();

    for (int i = 0; i < deviceCount; i++) {
      float t = devices[i]->temperature(sensors);
      bool  ok = SensorDevice::isValidReading(t);

      String data  = devices[i]->payload(t, rssi, ok);
      String topic = devices[i]->topic(config.appId);
      mqtt.publish(topic.c_str(), data.c_str());
      D_printf("Published: %s\n", topic.c_str());
    }
  }
}
