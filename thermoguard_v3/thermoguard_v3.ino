/*
 * Refactored Version (Iteration 6) – Mass Provisioning Edition
 * - REMOVED: All hardcoded SD_xxxx macros
 * - ADDED: Preferences (NVS) for DevEUI, AppKey, and Voltage Calibration
 * - ADDED: Serial Provisioning Mode (Waits for Python script if NVS is empty)
 * - OPTIMIZED: Uses RTC_DATA_ATTR to cache credentials across Deep Sleep (avoids NVS read overhead)
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <GyverMAX6675.h>
#include <Adafruit_BMP280.h>
#include <Preferences.h>

/* -------------------------- DEBUG MACROS -------------------------- */
#define DEBUG

#ifdef DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__);
#define D_print(...) Serial.print(__VA_ARGS__)
#define D_write(...) Serial.write(__VA_ARGS__)
#define D_println(...) Serial.println(__VA_ARGS__)
#define D_printf(...) Serial.printf(__VA_ARGS__)
#else
#define D_SerialBegin(...)
#define D_print(...)
#define D_write(...)
#define D_println(...)
#define D_printf(...)
#endif

/* -------------------------- PIN / HW DEFINES -------------------------- */
#define CLK_PIN_1 5  // Placas desarmadas -> 5
#define DATA_PIN_1 7 // Placas desarmadas -> 7
#define CS_PIN_1 6

#define VBAT_READ_PIN 1
#define ADC_CTRL_PIN 37
#define BATTERY_CHARGING_PIN 48
#define ADC_RESOLUTION 12
#define ADC_MAX_VOLTAGE 3.3
#define VOLTAGE_R1 390
#define VOLTAGE_R2 100

#define BATTERY_MIN_VOLTAGE 3200
#define BATTERY_MAX_VOLTAGE 4200
#define BATTERY_LOW_THRESHOLD 3400

#define TEMP_ERROR_VALUE -999.0
#define NUM_SENSORS 2

#define BATTERY_ADC_SAMPLES 3

/* -------------------------- DYNAMIC SLEEP CONFIG -------------------------- */
#define TEMP_THRESHOLD_LOW 20.0f
#define TEMP_THRESHOLD_HIGH 100.0f
#define SLEEP_TIME_LOW_MS 90000  // 1 min 30 sec
#define SLEEP_TIME_HIGH_MS 25000 // 25 sec

/* -------------------------- SENSOR OBJECTS -------------------------- */
GyverMAX6675<CLK_PIN_1, DATA_PIN_1, CS_PIN_1> tempSensor0;
Adafruit_BMP280 bmp;
bool bmpInitialized = false;

/* -------------------------- PERSISTENT DATA (RTC) -------------------------- */
/* These variables survive Deep Sleep but NOT a Reset/Power Cycle */

// Credentials (stored in RTC to avoid reading Flash on every wake up)
RTC_DATA_ATTR uint8_t devEui[8];
RTC_DATA_ATTR uint8_t appKey[16];
RTC_DATA_ATTR uint8_t appEui[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// Calibration Data
RTC_DATA_ATTR float calibrationMeasured = 1.0;
RTC_DATA_ATTR float calibrationReported = 1.0;

// System State
RTC_DATA_ATTR bool credentialsLoaded = false; // False on Reset, True on Wake
RTC_DATA_ATTR bool firstrun = true;
RTC_DATA_ATTR uint32_t wakeCounter = 0;

// Sensor State
RTC_DATA_ATTR float lastValidTemperature[NUM_SENSORS] = {TEMP_ERROR_VALUE, TEMP_ERROR_VALUE};
RTC_DATA_ATTR unsigned long successfulReadings[NUM_SENSORS] = {0, 0};
RTC_DATA_ATTR unsigned long failedReadings[NUM_SENSORS] = {0, 0};
RTC_DATA_ATTR bool sensorConnected[NUM_SENSORS] = {false, false};
RTC_DATA_ATTR bool lastReadSuccessful[NUM_SENSORS] = {false, false};

/* -------------------------- RUNTIME VARS -------------------------- */
float currentTemperature[NUM_SENSORS] = {0.0, 0.0};
bool measurementsTakenThisCycle = false;
int clkPins[NUM_SENSORS] = {CLK_PIN_1, -1};
int dataPins[NUM_SENSORS] = {DATA_PIN_1, -1};
int csPins[NUM_SENSORS] = {CS_PIN_1, -1};

uint16_t batteryVoltage = 0;
uint8_t batteryPercentage = 0;
bool batteryLow = false;
bool batteryCharging = false;

/* -------------------------- LORAWAN CONFIG -------------------------- */
uint16_t userChannelsMask[6] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_AU915;
DeviceClass_t loraWanClass = CLASS_A;
uint32_t appTxDutyCycle = SLEEP_TIME_HIGH_MS;
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = true;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

/* -------------------------- FORWARD DECLARATIONS -------------------------- */
void loadCredentials();
void runProvisioningMode();
void hexStringToBytes(String hexStr, uint8_t *output, size_t len);
// ... existing debug declarations ...
void debugPrintStartupBanner();
void debugPrintResumeBanner();
void debugPrintPinConfiguration();
void debugSystemInitialized(bool first);
void debugBatteryRead(uint16_t raw, float floatVoltage);
void debugBatteryInfo();
void debugReadTempHeader();
void debugReadTempFooter();
void debugSingleTempResult(int sensorIndex, bool success, float tempC);
void debugInvalidTemp(int sensorIndex, float temp, const String &reason);
void debugSensorError(int sensorIndex, const String &err);
void debugSensorTestStart();
void debugSensorTestAttempt(int sensorIndex, int attempt, bool success, float temp);
void debugSensorTestFailure(int sensorIndex);
void debugSensorTestSummary();
void debugLoRaSendHeader();
void debugLoRaSendDetails(float t0, float t1);
void debugLoRaSendFooter();
void debugLoRaPayload(float t0, float t1, uint16_t battMv, uint8_t battPct, uint8_t statusByte);
void debugUsingLastValid(int sensorIndex);
void debugSendingErrorValue(int sensorIndex);
void debugLoRaStatus(const String &state);
uint32_t calculateDynamicDutyCycle(float temp);
// Forward declare helpers defined later
uint16_t readBatteryVoltage();
void readTemperatures();
void readSingleTemperature(int sensorIndex);
bool validateTemperature(float temp, int sensorIndex);
void handleSensorError(int sensorIndex, String errorMessage);
void testSensorConnections();
void testSingleSensor(int sensorIndex);

/* -------------------------- SETUP / LOOP -------------------------- */

void setup()
{
  Serial.begin(115200);

  // 1. INIT HARDWARE EARLY
  // We do this first so we can read the battery if we enter Provisioning Mode
  pinMode(ADC_CTRL_PIN, OUTPUT);
  pinMode(VBAT_READ_PIN, INPUT);
  pinMode(BATTERY_CHARGING_PIN, INPUT_PULLDOWN);
  analogReadResolution(ADC_RESOLUTION);

  // 2. CREDENTIAL HANDLING
  // If we are waking from deep sleep, 'credentialsLoaded' will be true.
  // If we are booting up fresh (Reset/Power), it will be false.
  if (!credentialsLoaded)
  {
    loadCredentials();
    // If loadCredentials() fails to find data, it enters Provisioning Mode (Blocking)
    credentialsLoaded = true;
  }

  // 3. Standard Logic
  if (firstrun)
  {
    debugPrintStartupBanner();
  }
  else
  {
    debugPrintResumeBanner();
  }

  if (firstrun)
  {
    debugPrintPinConfiguration();
  }

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  bmpInitialized = bmp.begin(0x76) || bmp.begin(0x77);
  if (bmpInitialized)
  {
    bmp.setSampling(
        Adafruit_BMP280::MODE_FORCED,
        Adafruit_BMP280::SAMPLING_X1,
        Adafruit_BMP280::SAMPLING_X1,
        Adafruit_BMP280::FILTER_X16,
        Adafruit_BMP280::STANDBY_MS_500);
  }
  else
  {
    sensorConnected[1] = false;
    lastReadSuccessful[1] = false;
  }

  debugSystemInitialized(firstrun);

  if (firstrun)
  {
    testSensorConnections();
    readBatteryVoltage();
    debugBatteryInfo();
  }

  wakeCounter++;
  firstrun = false;
}
void loop()
{
  handleLoRaWANStateMachine();
}

/* -------------------------- PROVISIONING / NVS LOGIC -------------------------- */

void loadCredentials()
{
  Preferences prefs;
  prefs.begin("lorawan", false); // Namespace "lorawan", read/write

  // Check if keys exist
  if (prefs.isKey("devEui") && prefs.isKey("appKey"))
  {
    D_println("NVS: Credentials found. Loading into RTC...");

    prefs.getBytes("devEui", devEui, 8);
    prefs.getBytes("appKey", appKey, 16);

    // Load calibration if present, else default to 1.0
    calibrationMeasured = prefs.getFloat("vMeas", 1.0);
    calibrationReported = prefs.getFloat("vRep", 1.0);

    if (calibrationMeasured <= 0)
      calibrationMeasured = 1.0;
    if (calibrationReported <= 0)
      calibrationReported = 1.0;

    prefs.end();
  }
  else
  {
    D_println("NVS: No credentials found!");
    prefs.end();
    // Block execution and wait for Python script
    runProvisioningMode();
  }
}

void runProvisioningMode()
{
  // 1. Force Neutral Calibration for this read
  calibrationMeasured = 1.0;
  calibrationReported = 1.0;

  // 2. Read the battery now
  readBatteryVoltage();

  // 3. Print info for the user/operator
  Serial.println("--- PROVISIONING REQUIRED ---");
  Serial.print("PRE-PROVISION VOLTAGE (Raw): ");
  Serial.print(batteryVoltage / 1000.0, 3);
  Serial.println(" V");
  Serial.println("-----------------------------");
  Serial.println("Send: DEV_EUI,APP_KEY,V_MEAS,V_REP");
  Serial.println("Waiting...");

  while (true)
  {
    if (Serial.available())
    {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.length() > 20)
      {
        // Format: DEVEUI,APPKEY,V_MEAS,V_REP
        int c1 = line.indexOf(',');
        int c2 = line.indexOf(',', c1 + 1);
        int c3 = line.indexOf(',', c2 + 1);

        if (c1 > 0 && c2 > 0)
        {
          String sDevEui = line.substring(0, c1);
          String sAppKey = line.substring(c1 + 1, c2);
          String sVMeas = (c3 > 0) ? line.substring(c2 + 1, c3) : line.substring(c2 + 1);
          String sVRep = (c3 > 0) ? line.substring(c3 + 1) : "1.0";

          // Parse Floats
          float fMeas = sVMeas.toFloat();
          float fRep = sVRep.toFloat();

          // Parse Bytes
          uint8_t tempDevEui[8];
          uint8_t tempAppKey[16];
          hexStringToBytes(sDevEui, tempDevEui, 8);
          hexStringToBytes(sAppKey, tempAppKey, 16);

          // Save to NVS
          Preferences prefs;
          prefs.begin("lorawan", false);
          prefs.putBytes("devEui", tempDevEui, 8);
          prefs.putBytes("appKey", tempAppKey, 16);
          prefs.putFloat("vMeas", fMeas);
          prefs.putFloat("vRep", fRep);
          prefs.end();

          Serial.println("PROVISIONING COMPLETE. REBOOTING...");
          delay(1000);
          ESP.restart();
        }
      }
    }
    delay(100);
  }
}
void hexStringToBytes(String hexStr, uint8_t *output, size_t len)
{
  hexStr.replace(" ", ""); // remove spaces if any
  hexStr.replace("0x", "");
  for (size_t i = 0; i < len; i++)
  {
    if (i * 2 + 1 < hexStr.length())
    {
      char buffer[3];
      buffer[0] = hexStr[i * 2];
      buffer[1] = hexStr[i * 2 + 1];
      buffer[2] = 0;
      output[i] = (uint8_t)strtol(buffer, NULL, 16);
    }
    else
    {
      output[i] = 0;
    }
  }
}

/* -------------------------- BATTERY -------------------------- */
uint16_t readBatteryVoltage()
{
  const int adcMax = (1 << ADC_RESOLUTION) - 1;
  // USE DYNAMIC CALIBRATION VARS HERE:
  const float factor = (ADC_MAX_VOLTAGE / adcMax) * ((VOLTAGE_R1 + VOLTAGE_R2) / (float)VOLTAGE_R2) * (calibrationMeasured / calibrationReported);

  uint32_t sum = 0;
  int samples[BATTERY_ADC_SAMPLES];

  digitalWrite(ADC_CTRL_PIN, HIGH);
  for (int i = 0; i < BATTERY_ADC_SAMPLES; i++)
  {
    samples[i] = analogRead(VBAT_READ_PIN);
    sum += (uint32_t)samples[i];
  }
  digitalWrite(ADC_CTRL_PIN, LOW);

  batteryCharging = (digitalRead(BATTERY_CHARGING_PIN) == HIGH);

  float avgRaw = sum / (float)BATTERY_ADC_SAMPLES;
  float floatVoltage = factor * avgRaw;

  batteryVoltage = (uint16_t)(floatVoltage * 1000.0f); // mV

  if (batteryVoltage >= BATTERY_MAX_VOLTAGE)
  {
    batteryPercentage = 100;
  }
  else if (batteryVoltage <= BATTERY_MIN_VOLTAGE)
  {
    batteryPercentage = 0;
  }
  else
  {
    uint16_t voltageRange = BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE;
    uint16_t currentRange = batteryVoltage - BATTERY_MIN_VOLTAGE;
    batteryPercentage = (uint8_t)((currentRange * 100) / voltageRange);
  }
  batteryLow = (batteryVoltage < BATTERY_LOW_THRESHOLD);

  debugBatteryRead((uint16_t)avgRaw, floatVoltage);
  return batteryVoltage;
}

/* -------------------------- TEMPERATURE -------------------------- */
void readTemperatures()
{
  debugReadTempHeader();
  readSingleTemperature(0);
  readSingleTemperature(1);
  debugReadTempFooter();
}

void readSingleTemperature(int sensorIndex)
{
  bool readSuccess = false;
  float newTemp = 0.0f;

  if (sensorIndex == 0)
  {
    readSuccess = tempSensor0.readTemp();
    if (readSuccess)
      newTemp = tempSensor0.getTemp();
  }
  else if (sensorIndex == 1)
  {
    if (bmpInitialized)
    {
      if (bmp.takeForcedMeasurement())
      {
        newTemp = bmp.readTemperature();
        readSuccess = !isnan(newTemp);
      }
      else
      {
        readSuccess = false;
      }
    }
    else
    {
      readSuccess = false;
    }
  }

  if (readSuccess)
  {
    if (validateTemperature(newTemp, sensorIndex))
    {
      currentTemperature[sensorIndex] = newTemp;
      lastValidTemperature[sensorIndex] = newTemp;
      sensorConnected[sensorIndex] = true;
      lastReadSuccessful[sensorIndex] = true;
      successfulReadings[sensorIndex]++;
      debugSingleTempResult(sensorIndex, true, newTemp);
    }
    else
    {
      failedReadings[sensorIndex]++;
      lastReadSuccessful[sensorIndex] = false;
      debugInvalidTemp(sensorIndex, newTemp, "Invalid temperature");
      handleSensorError(sensorIndex, "Invalid temperature reading");
    }
  }
  else
  {
    failedReadings[sensorIndex]++;
    lastReadSuccessful[sensorIndex] = false;
    sensorConnected[sensorIndex] = false;
    debugSingleTempResult(sensorIndex, false, TEMP_ERROR_VALUE);
    handleSensorError(sensorIndex, "Sensor read failed");
  }
}

bool validateTemperature(float temp, int sensorIndex)
{
  if (temp < -50 || temp > 1000)
  {
    debugInvalidTemp(sensorIndex, temp, "Out of range");
    return false;
  }
  return true;
}

void handleSensorError(int sensorIndex, String errorMessage)
{
  debugSensorError(sensorIndex, errorMessage);
  if (lastValidTemperature[sensorIndex] != TEMP_ERROR_VALUE && successfulReadings[sensorIndex] > 0)
  {
    currentTemperature[sensorIndex] = lastValidTemperature[sensorIndex];
  }
  else
  {
    currentTemperature[sensorIndex] = TEMP_ERROR_VALUE;
  }
}

void testSensorConnections()
{
  debugSensorTestStart();
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    testSingleSensor(i);
  }
  debugSensorTestSummary();
}

void testSingleSensor(int sensorIndex)
{
  const int attempts = 3;
  bool success = false;

  for (int i = 0; i < attempts; i++)
  {
    bool readOk = false;
    float temp = 0.0f;

    if (sensorIndex == 0)
    {
      readOk = tempSensor0.readTemp();
      if (readOk)
        temp = tempSensor0.getTemp();
    }
    else
    {
      if (!bmpInitialized)
      {
        bmpInitialized = bmp.begin(0x76) || bmp.begin(0x77);
        if (bmpInitialized)
        {
          bmp.setSampling(
              Adafruit_BMP280::MODE_FORCED,
              Adafruit_BMP280::SAMPLING_X2,
              Adafruit_BMP280::SAMPLING_X16,
              Adafruit_BMP280::FILTER_X16,
              Adafruit_BMP280::STANDBY_MS_500);
        }
      }
      if (bmpInitialized && bmp.takeForcedMeasurement())
      {
        temp = bmp.readTemperature();
        readOk = !isnan(temp);
      }
    }

    if (readOk && validateTemperature(temp, sensorIndex))
    {
      currentTemperature[sensorIndex] = temp;
      lastValidTemperature[sensorIndex] = temp;
      sensorConnected[sensorIndex] = true;
      lastReadSuccessful[sensorIndex] = true;
      debugSensorTestAttempt(sensorIndex, i + 1, true, temp);
      success = true;
      break;
    }
    else
    {
      debugSensorTestAttempt(sensorIndex, i + 1, false, temp);
    }
  }

  if (!success)
  {
    sensorConnected[sensorIndex] = false;
    lastReadSuccessful[sensorIndex] = false;
    debugSensorTestFailure(sensorIndex);
  }
}

/* -------------------------- LORAWAN HELPERS -------------------------- */
float getTemperatureToSend(int sensorIndex)
{
  if (sensorConnected[sensorIndex] && currentTemperature[sensorIndex] != TEMP_ERROR_VALUE)
  {
    return currentTemperature[sensorIndex];
  }
  else if (lastValidTemperature[sensorIndex] != TEMP_ERROR_VALUE)
  {
    debugUsingLastValid(sensorIndex);
    return lastValidTemperature[sensorIndex];
  }
  else
  {
    debugSendingErrorValue(sensorIndex);
    return TEMP_ERROR_VALUE;
  }
}

/* -------------------------- DYNAMIC DUTY CYCLE -------------------------- */
uint32_t calculateDynamicDutyCycle(float temp)
{
  if (temp <= TEMP_THRESHOLD_LOW)
  {
    return SLEEP_TIME_LOW_MS;
  }
  else if (temp >= TEMP_THRESHOLD_HIGH)
  {
    return SLEEP_TIME_HIGH_MS;
  }
  else
  {
    float slope = (float)(SLEEP_TIME_HIGH_MS - SLEEP_TIME_LOW_MS) / (TEMP_THRESHOLD_HIGH - TEMP_THRESHOLD_LOW);
    float sleepTime = slope * (temp - TEMP_THRESHOLD_LOW) + SLEEP_TIME_LOW_MS;
    return (uint32_t)sleepTime;
  }
}

String getLoRaWANStatusString()
{
  switch (deviceState)
  {
  case DEVICE_STATE_INIT:
    return "Initializing";
  case DEVICE_STATE_JOIN:
    return "Joining Network";
  case DEVICE_STATE_SEND:
    return "Sending Data";
  case DEVICE_STATE_CYCLE:
    return "Waiting";
  case DEVICE_STATE_SLEEP:
    return "Sleeping";
  default:
    return "Unknown State";
  }
}

static void prepareTxFrame(uint8_t port, float temperature0, float temperature1)
{
  union
  {
    struct
    {
      float temperature0;
      float temperature1;
      uint16_t batteryVoltage;
      uint8_t batteryPercentage;
      uint8_t sensorStatus;
    } data;
    uint8_t bytes[12];
  } payload;

  payload.data.temperature0 = temperature0;
  payload.data.temperature1 = temperature1;
  payload.data.batteryVoltage = batteryVoltage;
  payload.data.batteryPercentage = batteryPercentage;

  payload.data.sensorStatus = 0;
  if (sensorConnected[0])
    payload.data.sensorStatus |= 0x01;
  if (lastReadSuccessful[0])
    payload.data.sensorStatus |= 0x02;
  if (sensorConnected[1])
    payload.data.sensorStatus |= 0x04;
  if (lastReadSuccessful[1])
    payload.data.sensorStatus |= 0x08;
  if (batteryLow)
    payload.data.sensorStatus |= 0x10;
  if (successfulReadings[0] > failedReadings[0])
    payload.data.sensorStatus |= 0x20;
  if (successfulReadings[1] > failedReadings[1])
    payload.data.sensorStatus |= 0x40;
  if (batteryCharging)
    payload.data.sensorStatus |= 0x80;

  appDataSize = 12;
  for (int i = 0; i < appDataSize; i++)
  {
    appData[i] = payload.bytes[i];
  }

  debugLoRaPayload(temperature0, temperature1, batteryVoltage, batteryPercentage, payload.data.sensorStatus);
}

void handleLoRaWANStateMachine()
{
  switch (deviceState)
  {
  case DEVICE_STATE_INIT:
  {
#if (LORAWAN_DEVEUI_AUTO)
    LoRaWAN.generateDeveuiByChipID();
#endif
    LoRaWAN.init(loraWanClass, loraWanRegion);
    LoRaWAN.setDefaultDR(3);
    debugLoRaStatus("LoRaWAN initialized");
    break;
  }
  case DEVICE_STATE_JOIN:
  {
    debugLoRaStatus("Attempting to join network");
    LoRaWAN.join();
    break;
  }
  case DEVICE_STATE_SEND:
  {
    debugLoRaSendHeader();

    if (!measurementsTakenThisCycle)
    {
      readBatteryVoltage();
      readTemperatures();
      measurementsTakenThisCycle = true;
    }

    float t0 = getTemperatureToSend(0);
    float t1 = getTemperatureToSend(1);
    debugLoRaSendDetails(t0, t1);

    // --- DYNAMIC SLEEP CALCULATION ---
    float maxTemp = -999.0;
    bool validTempFound = false;

    if (t0 != TEMP_ERROR_VALUE)
    {
      maxTemp = t0;
      validTempFound = true;
    }
    if (t1 != TEMP_ERROR_VALUE)
    {
      if (!validTempFound || t1 > maxTemp)
      {
        maxTemp = t1;
        validTempFound = true;
      }
    }

    if (validTempFound)
    {
      appTxDutyCycle = calculateDynamicDutyCycle(maxTemp);
      D_print("Dynamic Sleep: Max Temp ");
      D_print(maxTemp);
      D_print("C -> Interval ");
      D_print(appTxDutyCycle);
      D_println(" ms");
    }
    else
    {
      appTxDutyCycle = SLEEP_TIME_HIGH_MS;
      D_println("Dynamic Sleep: No valid sensors. Defaulting to slow interval.");
    }

    prepareTxFrame(appPort, t0, t1);
    LoRaWAN.send();
    deviceState = DEVICE_STATE_CYCLE;

    debugLoRaSendFooter();
    break;
  }
  case DEVICE_STATE_CYCLE:
  {
    txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
    LoRaWAN.cycle(txDutyCycleTime);
    deviceState = DEVICE_STATE_SLEEP;
    measurementsTakenThisCycle = false;
    break;
  }
  case DEVICE_STATE_SLEEP:
  {
    LoRaWAN.sleep(loraWanClass);
    break;
  }
  default:
  {
    deviceState = DEVICE_STATE_INIT;
    break;
  }
  }
}

/* -------------------------- DEBUG / PRINT FUNCTIONS -------------------------- */
/* -------------------------- DEBUG / PRINT FUNCTIONS -------------------------- */
void debugPrintStartupBanner()
{
  D_println();
  D_println(String('=', 75));
  D_println("  MAX6675 + BMP280 Temperature Sensors + Battery + LoRaWAN");
  D_println("  First Boot (Cold Start)");
  D_println("  Libraries: GyverMAX6675, Adafruit_BMP280 | Board: Heltec ESP32 V3");
  D_println(String('=', 75));
}

void debugPrintResumeBanner()
{
  D_println();
  D_println(String('-', 55));
  D_println("  Wake from Deep Sleep (Warm Start)");
  D_print("  Wake Counter: ");
  D_println(wakeCounter + 1);
  D_println(String('-', 55));
}

void debugPrintPinConfiguration()
{
  D_println("Sensor Configuration:");
  D_println("SENSOR 0 (MAX6675):");
  D_println("  CLK (SCK):     GPIO " + String(clkPins[0]));
  D_println("  DATA (SO):     GPIO " + String(dataPins[0]));
  D_println("  CS:            GPIO " + String(csPins[0]));
  D_println("SENSOR 1 (BMP280 over I2C):");
  D_println("  I2C SDA/SCL:   Board default (check Heltec V3 docs)");
  D_println("  Address tried: 0x76, 0x77");
  D_println("BATTERY:");
  D_println("  Battery ADC:   GPIO " + String(VBAT_READ_PIN));
  D_println("  ADC Control:   GPIO " + String(ADC_CTRL_PIN));
  D_println("  Charging Pin:  GPIO " + String(BATTERY_CHARGING_PIN));
  D_println();
}

void debugSystemInitialized(bool first)
{
  if (first)
  {
    D_println("System initialized (first run). Running diagnostics...");
  }
  else
  {
    D_println("System initialized (resume). Skipping full diagnostics.");
  }
}

void debugBatteryRead(uint16_t raw, float floatVoltage)
{
  D_print("[readBatteryVoltage] ADC(avg): ");
  D_println(raw);
  D_print("[readBatteryVoltage] Float: ");
  D_println(floatVoltage, 3);
  D_print("[readBatteryVoltage] milliVolts: ");
  D_println(batteryVoltage);
  D_print("[readBatteryVoltage] Percentage: ");
  D_print(batteryPercentage);
  D_print("%");
  if (batteryLow)
    D_print(" - LOW BATTERY WARNING!");
  if (batteryCharging)
    D_print(" - CHARGING");
  D_println();
}

void debugBatteryInfo()
{
  D_println();
  D_println(String('-', 50));
  D_println("BATTERY INFORMATION");
  D_println(String('-', 50));
  D_println("  Voltage: " + String(batteryVoltage / 1000.0, 2) + "V");
  D_println("  Percentage: " + String(batteryPercentage) + "%");
  D_println("  Status: " + String(batteryLow ? "LOW" : "GOOD"));
  D_println("  Charging: " + String(batteryCharging ? "YES" : "NO"));
  D_println("  Min Safe: " + String(BATTERY_MIN_VOLTAGE / 1000.0, 1) + "V");
  D_println("  Max Charge: " + String(BATTERY_MAX_VOLTAGE / 1000.0, 1) + "V");
  D_println("  Low Threshold: " + String(BATTERY_LOW_THRESHOLD / 1000.0, 1) + "V");
  D_println(String('-', 50));
  D_println();
}

void debugReadTempHeader()
{
  D_println("=== Reading MAX6675 (ch0) + BMP280 (ch1) ===");
}
void debugReadTempFooter()
{
  D_println("============================================");
}

void debugSingleTempResult(int sensorIndex, bool success, float tempC)
{
  if (success)
  {
    D_print("Sensor ");
    D_print(sensorIndex);
    D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
    D_print(": SUCCESS - ");
    D_print(tempC, 2);
    D_println("°C");
  }
  else
  {
    D_print("Sensor ");
    D_print(sensorIndex);
    D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
    D_println(": FAILED - Sensor error or disconnected");
  }
}

void debugInvalidTemp(int sensorIndex, float temp, const String &reason)
{
  D_print("Sensor ");
  D_print(sensorIndex);
  D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
  D_print(" INVALID ");
  D_print(temp, 2);
  D_print("°C - ");
  D_println(reason);
}

void debugSensorError(int sensorIndex, const String &err)
{
  D_print("ERROR: Sensor ");
  D_print(sensorIndex);
  D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
  D_print(" - ");
  D_println(err);
}

void debugSensorTestStart()
{
  D_println("Testing sensor connections...");
}

void debugSensorTestAttempt(int sensorIndex, int attempt, bool success, float temp)
{
  D_print("Sensor ");
  D_print(sensorIndex);
  D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
  D_print(" Test ");
  D_print(attempt);
  D_print("/3: ");
  if (success)
  {
    D_print("SUCCESS - Temperature: ");
    D_print(temp, 1);
    D_println("°C");
  }
  else
  {
    D_println("FAILED");
  }
}

void debugSensorTestFailure(int sensorIndex)
{
  if (sensorIndex == 0)
  {
    D_println("⚠ WARNING: Sensor 0 [MAX6675] test failed!");
    D_println("Check wiring:");
    D_println("  - VCC (3.3V)");
    D_println("  - GND");
    D_println("  - SCK GPIO " + String(clkPins[0]));
    D_println("  - SO  GPIO " + String(dataPins[0]));
    D_println("  - CS  GPIO " + String(csPins[0]));
    D_println("  - Thermocouple seating");
  }
  else
  {
    D_println("⚠ WARNING: Sensor 1 [BMP280] test failed!");
    D_println("Check I2C wiring:");
    D_println("  - 3V3 and GND");
    D_println("  - SDA / SCL (board defaults)");
    D_println("  - Address 0x76 or 0x77");
  }
}

void debugSensorTestSummary()
{
  D_println();
  D_println("--- Sensor Test Summary ---");
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    D_print("Sensor ");
    D_print(i);
    D_print(i == 0 ? " [MAX6675]" : " [BMP280]");
    D_print(": ");
    D_println(sensorConnected[i] ? "✓ WORKING" : "✗ FAILED");
  }
  D_println("---------------------------");
}

void debugLoRaSendHeader()
{
  D_println();
  D_println(String('*', 60));
  D_println("PREPARING LORAWAN TRANSMISSION");
  D_println(String('*', 60));
}

void debugLoRaSendDetails(float t0, float t1)
{
  D_println("MAX6675 (ch0): " + String(t0, 1) + "°C");
  D_println("BMP280  (ch1): " + String(t1, 1) + "°C");
  D_println("Battery: " + String(batteryVoltage / 1000.0, 2) + "V (" + String(batteryPercentage) + "%)");
  D_println("Charging: " + String(batteryCharging ? "Yes" : "No"));
  D_println("Wake Count: " + String(wakeCounter));
}

void debugLoRaSendFooter()
{
  D_println("LoRaWAN transmission queued/completed");
  D_println("Next Sleep Time: " + String(appTxDutyCycle) + " ms");
  D_println(String('*', 60));
  D_println();
}

void debugLoRaPayload(float t0, float t1, uint16_t battMv, uint8_t battPct, uint8_t statusByte)
{
  D_println("LoRaWAN Payload Details:");
  D_println("  MAX6675 temp: " + String(t0, 2) + "°C");
  D_println("  BMP280  temp: " + String(t1, 2) + "°C");
  D_println("  Battery Voltage: " + String(battMv / 1000.0, 2) + "V (" + String(battMv) + " mV)");
  D_println("  Battery Percentage: " + String(battPct) + "%");
  D_println("  MAX6675 Connected: " + String(sensorConnected[0] ? "Yes" : "No"));
  D_println("  BMP280  Connected: " + String(sensorConnected[1] ? "Yes" : "No"));
  D_println("  Battery Low: " + String(batteryLow ? "Yes" : "No"));
  D_println("  Charging: " + String((statusByte & 0x80) ? "Yes" : "No"));
  D_print("  Status Byte: 0x");
  if (statusByte < 0x10)
    D_print("0");
  D_println(String(statusByte, HEX));
  D_print("  Payload bytes: [");
  for (int i = 0; i < appDataSize; i++)
  {
    if (i > 0)
      D_print(", ");
    D_print("0x");
    if (appData[i] < 0x10)
      D_print("0");
    D_print(String(appData[i], HEX));
  }
  D_println("]");
}

void debugUsingLastValid(int sensorIndex)
{
  D_print("Sensor ");
  D_print(sensorIndex);
  D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
  D_println(" using last valid reading");
}

void debugSendingErrorValue(int sensorIndex)
{
  D_print("Sensor ");
  D_print(sensorIndex);
  D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
  D_println(" sending error value");
}

void debugLoRaStatus(const String &state)
{
  D_println("LORAWAN STATUS: " + state);
}

/* Required for LoRaWAN library persistence included above */
uint8_t nwkSKey[] = {0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85};
uint8_t appSKey[] = {0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67};
uint32_t devAddr = (uint32_t)0x007e6ae1;