/*
 * Refactored Version (Iteration 5) – Dynamic Sleep Time based on Temperature
 * - Uses Adafruit_BMP280 for the second "temperature" channel
 * - BMP280 temperature is sent as the second temperature in the LoRaWAN payload
 * - Keeps all prior deep-sleep handling, debug macros, and state machine
 * - No delays added; BMP in forced mode blocks only for measurement by library
 * - Averaged battery ADC reads (3 samples), no blocking delays
 * - Reading of BATTERY_CHARGING_PIN and sending status in flag bit 0x80
 * - ADDED: Dynamic transmission interval (Duty Cycle) based on temperature
 *   (20°C -> 90s, 100°C -> 25s, linear interpolation)
 *
 * Notes:
 * - I2C address is auto-tried (0x76 then 0x77). Adjust if your wiring uses a different address.
 * - Sensor 0 remains MAX6675. Sensor 1 is now BMP280.
 */

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <GyverMAX6675.h>
#include <Adafruit_BMP280.h>

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
#define CLK_PIN_1 5   //Placas desarmadas -> 5
#define DATA_PIN_1 7  // Placas desarmadas -> 7
#define CS_PIN_1 6

// MAX6675 #2 removed. BMP280 uses I2C

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

// Average 3 samples for ADC
#define BATTERY_ADC_SAMPLES 3

/* -------------------------- DYNAMIC SLEEP CONFIG -------------------------- */
#define TEMP_THRESHOLD_LOW 20.0f
#define TEMP_THRESHOLD_HIGH 100.0f
#define SLEEP_TIME_LOW_MS 90000   // 1 min 30 sec
#define SLEEP_TIME_HIGH_MS 25000  // 25 sec

/* -------------------------- SENSOR OBJECTS -------------------------- */
GyverMAX6675<CLK_PIN_1, DATA_PIN_1, CS_PIN_1> tempSensor0;
Adafruit_BMP280 bmp;  // I2C BMP280 for second temperature
bool bmpInitialized = false;

/* -------------------------- DEVICE DEFINITION -------------------------- */
#define SD_0012
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
/* -------------------------- LORAWAN PARAMETERS -------------------------- */

#ifdef SD_0001
#define CALIBRATION_MEASURED 4.10
#define CALIBRATION_REPORTED 3.818
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x28, 0xBF };                                                  //70B3D57ED00728BF
uint8_t appKey[] = { 0x01, 0x81, 0xDD, 0x9E, 0xE6, 0x12, 0xFD, 0xDF, 0x3A, 0xD2, 0x6F, 0x9C, 0xBF, 0xD4, 0x12, 0xA0 };  //0181dd9ee612fddf3ad26f9cbfd412a0
#endif

#ifdef SD_0002
#define CALIBRATION_MEASURED 3.98
#define CALIBRATION_REPORTED 3.699
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x28, 0xE3 };
uint8_t appKey[] = { 0xFB, 0x39, 0xAE, 0x4B, 0xDE, 0x08, 0xE7, 0x32, 0x95, 0x40, 0xEB, 0xFE, 0x4C, 0xC3, 0x41, 0x4E };  // fb39ae4bde08e7329540ebfe4cc3414e
#endif

#ifdef SD_0003
#define CALIBRATION_MEASURED 4.11
#define CALIBRATION_REPORTED 3.61
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x28, 0xE4 };
uint8_t appKey[] = { 0x75, 0x4A, 0x7D, 0x10, 0x64, 0x61, 0x1B, 0xEA, 0xFF, 0x3E, 0xDF, 0x09, 0xE6, 0x13, 0x2E, 0x25 };  // 754a7d1064611beaff3edf09e6132e25
#endif

#ifdef SD_0004
#define CALIBRATION_MEASURED 4.16
#define CALIBRATION_REPORTED 3.86
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x28, 0xE5 };
uint8_t appKey[] = { 0x3E, 0x74, 0xB6, 0x31, 0xBF, 0xF9, 0xE1, 0xA5, 0x24, 0x19, 0x0E, 0xCE, 0x8E, 0xAB, 0x51, 0xE7 };  // 3e74b631bff9e1a524190ece8eab51e7
#endif

#ifdef SD_0005
#define CALIBRATION_MEASURED 4.18
#define CALIBRATION_REPORTED 3.86
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x28, 0xE6 };
uint8_t appKey[] = { 0x14, 0x70, 0xEA, 0x97, 0xAC, 0x72, 0x04, 0x09, 0x7A, 0xD9, 0xA4, 0xBD, 0x83, 0xA8, 0xF3, 0x99 };  // 1470ea97ac7204097ad9a4bd83a8f399
#endif

#ifdef SD_0006
#define CALIBRATION_MEASURED 4.18
#define CALIBRATION_REPORTED 3.78
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x28, 0xE7 };
uint8_t appKey[] = { 0x27, 0x6A, 0x0A, 0x8B, 0xD1, 0xD9, 0x18, 0xDA, 0x9B, 0xD3, 0xC3, 0x9A, 0xB6, 0xBB, 0x6C, 0xF1 };  // 276a0a8bd1d918da9bd3c39ab6bb6cf1
#endif

#ifdef SD_0009
#define CALIBRATION_MEASURED 4.17
#define CALIBRATION_REPORTED 3.72
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x28, 0xEA };
uint8_t appKey[] = { 0x7D, 0x2D, 0xDB, 0xBA, 0xDD, 0xF9, 0xB0, 0xDF, 0x59, 0x01, 0x82, 0xEF, 0xE3, 0x57, 0xC3, 0x87 };  // 4318608d344502be5ece9241b18b5856
#endif

#ifdef SD_0010
#define CALIBRATION_MEASURED 4.17
#define CALIBRATION_REPORTED 3.83
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x28, 0xEB };
uint8_t appKey[] = { 0x4F, 0xF7, 0x7F, 0x20, 0x46, 0x77, 0x0B, 0xBB, 0xC6, 0x69, 0xC2, 0x88, 0x91, 0x49, 0x6E, 0xC3 };  // 4318608d344502be5ece9241b18b5856
#endif

#ifdef SD_0011
#define CALIBRATION_MEASURED 4.17
#define CALIBRATION_REPORTED 3.85
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x28, 0xEC };
uint8_t appKey[] = { 0x43, 0x18, 0x60, 0x8D, 0x34, 0x45, 0x02, 0xBE, 0x5E, 0xCE, 0x92, 0x41, 0xB1, 0x8B, 0x58, 0x56 };  // 4318608d344502be5ece9241b18b5856
#endif

#ifdef SD_0012
#define CALIBRATION_MEASURED 4.17
#define CALIBRATION_REPORTED 3.87
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x28, 0xED };
uint8_t appKey[] = { 0x66, 0x32, 0x34, 0xA0, 0xFC, 0x88, 0x29, 0x0A, 0x62, 0xBD, 0x4A, 0x05, 0x0E, 0x8B, 0x59, 0x63 };  // 4318608d344502be5ece9241b18b5856
#endif

#ifdef SD_0013
#define CALIBRATION_MEASURED 4.17
#define CALIBRATION_REPORTED 3.82
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x07, 0x28, 0xEE };
uint8_t appKey[] = { 0x93, 0x80, 0x80, 0x27, 0x8D, 0x02, 0x89, 0x37, 0x4F, 0xDD, 0x4C, 0x7B, 0x25, 0x86, 0x36, 0x5C };  // 4318608d344502be5ece9241b18b5856
#endif
/* -------------------------- LORAWAN CONFIG -------------------------- */
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };
LoRaMacRegion_t loraWanRegion = LORAMAC_REGION_AU915;
DeviceClass_t loraWanClass = CLASS_A;
uint32_t appTxDutyCycle = SLEEP_TIME_HIGH_MS;  // Default to slower rate
bool overTheAirActivation = true;
bool loraWanAdr = true;
bool isTxConfirmed = true;
uint8_t appPort = 2;
uint8_t confirmedNbTrials = 4;

/* -------------------------- RUNTIME STATE -------------------------- */
/* Current run (RAM only) */
float currentTemperature[NUM_SENSORS] = { 0.0, 0.0 };

/* Persisted across deep sleep (RTC fast memory) */
RTC_DATA_ATTR float lastValidTemperature[NUM_SENSORS] = { TEMP_ERROR_VALUE, TEMP_ERROR_VALUE };
RTC_DATA_ATTR unsigned long successfulReadings[NUM_SENSORS] = { 0, 0 };
RTC_DATA_ATTR unsigned long failedReadings[NUM_SENSORS] = { 0, 0 };
RTC_DATA_ATTR bool sensorConnected[NUM_SENSORS] = { false, false };
RTC_DATA_ATTR bool lastReadSuccessful[NUM_SENSORS] = { false, false };
RTC_DATA_ATTR uint32_t wakeCounter = 0;  // Counts wake cycles
/* -------------------------- FIRST RUN FLAG -------------------------- */
RTC_DATA_ATTR bool firstrun = true;

/* Non-persisted helper flags */
bool measurementsTakenThisCycle = false;

/* Pins arrays (const-like) for debug of MAX sensor only */
int clkPins[NUM_SENSORS] = { CLK_PIN_1, -1 };
int dataPins[NUM_SENSORS] = { DATA_PIN_1, -1 };
int csPins[NUM_SENSORS] = { CS_PIN_1, -1 };

/* Battery (recomputed per cycle) */
uint16_t batteryVoltage = 0;
uint8_t batteryPercentage = 0;
bool batteryLow = false;
bool batteryCharging = false;

/* Forward declarations (debug + core) */
void debugPrintStartupBanner();
void debugPrintResumeBanner();
void debugPrintPinConfiguration();
void debugSystemInitialized(bool first);
void debugBatteryRead(uint16_t raw, float floatVoltage);
void debugBatteryInfo();
void debugReadTempHeader();
void debugReadTempFooter();
void debugSingleTempResult(int sensorIndex, bool success, float tempC);
void debugInvalidTemp(int sensorIndex, float temp, const String& reason);
void debugSensorError(int sensorIndex, const String& err);
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
void debugLoRaStatus(const String& state);
uint32_t calculateDynamicDutyCycle(float temp);

/* -------------------------- SETUP / LOOP -------------------------- */
void setup() {
  D_SerialBegin(115200);

  if (firstrun) {
    debugPrintStartupBanner();
  } else {
    debugPrintResumeBanner();
  }

  pinMode(ADC_CTRL_PIN, OUTPUT);
  pinMode(VBAT_READ_PIN, INPUT);
  pinMode(BATTERY_CHARGING_PIN, INPUT_PULLDOWN);
  analogReadResolution(ADC_RESOLUTION);

  if (firstrun) {
    debugPrintPinConfiguration();
  }

  // Initialize MCU/LoRaWAN (also sets up board subsystems)
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // Initialize BMP280 (try both common I2C addresses)
  bmpInitialized = bmp.begin(0x76) || bmp.begin(0x77);
  if (bmpInitialized) {
    bmp.setSampling(
      Adafruit_BMP280::MODE_FORCED,
      Adafruit_BMP280::SAMPLING_X1,  // temperature oversampling
      Adafruit_BMP280::SAMPLING_X1,  // pressure oversampling (not used for TX here)
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_500);
  } else {
    sensorConnected[1] = false;
    lastReadSuccessful[1] = false;
  }

  debugSystemInitialized(firstrun);

  if (firstrun) {
    testSensorConnections();
    readBatteryVoltage();
    debugBatteryInfo();
  }

  wakeCounter++;
  firstrun = false;  // Mark subsequent boots as resumes
}

void loop() {
  handleLoRaWANStateMachine();
}

/* -------------------------- BATTERY -------------------------- */
// Averaged version (3 samples), no delays
uint16_t readBatteryVoltage() {
  const int adcMax = (1 << ADC_RESOLUTION) - 1;
  const float factor = (ADC_MAX_VOLTAGE / adcMax) * ((VOLTAGE_R1 + VOLTAGE_R2) / (float)VOLTAGE_R2) * (CALIBRATION_MEASURED / CALIBRATION_REPORTED);

  uint32_t sum = 0;
  int samples[BATTERY_ADC_SAMPLES];

  digitalWrite(ADC_CTRL_PIN, HIGH);
  for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
    samples[i] = analogRead(VBAT_READ_PIN);
    sum += (uint32_t)samples[i];
  }
  digitalWrite(ADC_CTRL_PIN, LOW);

  batteryCharging = (digitalRead(BATTERY_CHARGING_PIN) == HIGH);

  float avgRaw = sum / (float)BATTERY_ADC_SAMPLES;
  float floatVoltage = factor * avgRaw;

  batteryVoltage = (uint16_t)(floatVoltage * 1000.0f);  // mV

  if (batteryVoltage >= BATTERY_MAX_VOLTAGE) {
    batteryPercentage = 100;
  } else if (batteryVoltage <= BATTERY_MIN_VOLTAGE) {
    batteryPercentage = 0;
  } else {
    uint16_t voltageRange = BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE;
    uint16_t currentRange = batteryVoltage - BATTERY_MIN_VOLTAGE;
    batteryPercentage = (uint8_t)((currentRange * 100) / voltageRange);
  }
  batteryLow = (batteryVoltage < BATTERY_LOW_THRESHOLD);

#ifdef DEBUG
  D_print("[readBatteryVoltage] Samples: ");
  for (int i = 0; i < BATTERY_ADC_SAMPLES; i++) {
    if (i) D_print(", ");
    D_print(samples[i]);
  }
  D_println();
#endif

  debugBatteryRead((uint16_t)avgRaw, floatVoltage);
  return batteryVoltage;
}

/* -------------------------- TEMPERATURE -------------------------- */
void readTemperatures() {
  debugReadTempHeader();
  readSingleTemperature(0);  // MAX6675
  readSingleTemperature(1);  // BMP280
  debugReadTempFooter();
}

void readSingleTemperature(int sensorIndex) {
  bool readSuccess = false;
  float newTemp = 0.0f;

  if (sensorIndex == 0) {
    // MAX6675
    readSuccess = tempSensor0.readTemp();
    if (readSuccess) newTemp = tempSensor0.getTemp();
  } else if (sensorIndex == 1) {
    // BMP280 temperature
    if (bmpInitialized) {
      if (bmp.takeForcedMeasurement()) {
        newTemp = bmp.readTemperature();
        readSuccess = !isnan(newTemp);
      } else {
        readSuccess = false;
      }
    } else {
      readSuccess = false;
    }
  }

  if (readSuccess) {
    if (validateTemperature(newTemp, sensorIndex)) {
      currentTemperature[sensorIndex] = newTemp;
      lastValidTemperature[sensorIndex] = newTemp;
      sensorConnected[sensorIndex] = true;
      lastReadSuccessful[sensorIndex] = true;
      successfulReadings[sensorIndex]++;
      debugSingleTempResult(sensorIndex, true, newTemp);
    } else {
      failedReadings[sensorIndex]++;
      lastReadSuccessful[sensorIndex] = false;
      debugInvalidTemp(sensorIndex, newTemp, "Invalid temperature");
      handleSensorError(sensorIndex, "Invalid temperature reading");
    }
  } else {
    failedReadings[sensorIndex]++;
    lastReadSuccessful[sensorIndex] = false;
    sensorConnected[sensorIndex] = false;
    debugSingleTempResult(sensorIndex, false, TEMP_ERROR_VALUE);
    handleSensorError(sensorIndex, "Sensor read failed");
  }
}

bool validateTemperature(float temp, int sensorIndex) {
  if (temp < -50 || temp > 1000) {
    debugInvalidTemp(sensorIndex, temp, "Out of range");
    return false;
  }
  return true;
}

void handleSensorError(int sensorIndex, String errorMessage) {
  debugSensorError(sensorIndex, errorMessage);
  if (lastValidTemperature[sensorIndex] != TEMP_ERROR_VALUE && successfulReadings[sensorIndex] > 0) {
    currentTemperature[sensorIndex] = lastValidTemperature[sensorIndex];
  } else {
    currentTemperature[sensorIndex] = TEMP_ERROR_VALUE;
  }
}

void testSensorConnections() {
  debugSensorTestStart();
  for (int i = 0; i < NUM_SENSORS; i++) {
    testSingleSensor(i);
  }
  debugSensorTestSummary();
}

void testSingleSensor(int sensorIndex) {
  const int attempts = 3;
  bool success = false;

  for (int i = 0; i < attempts; i++) {
    bool readOk = false;
    float temp = 0.0f;

    if (sensorIndex == 0) {
      readOk = tempSensor0.readTemp();
      if (readOk) temp = tempSensor0.getTemp();
    } else {
      // BMP280 init if not done yet (try once here as fallback)
      if (!bmpInitialized) {
        bmpInitialized = bmp.begin(0x76) || bmp.begin(0x77);
        if (bmpInitialized) {
          bmp.setSampling(
            Adafruit_BMP280::MODE_FORCED,
            Adafruit_BMP280::SAMPLING_X2,
            Adafruit_BMP280::SAMPLING_X16,
            Adafruit_BMP280::FILTER_X16,
            Adafruit_BMP280::STANDBY_MS_500);
        }
      }
      if (bmpInitialized && bmp.takeForcedMeasurement()) {
        temp = bmp.readTemperature();
        readOk = !isnan(temp);
      }
    }

    if (readOk && validateTemperature(temp, sensorIndex)) {
      currentTemperature[sensorIndex] = temp;
      lastValidTemperature[sensorIndex] = temp;
      sensorConnected[sensorIndex] = true;
      lastReadSuccessful[sensorIndex] = true;
      debugSensorTestAttempt(sensorIndex, i + 1, true, temp);
      success = true;
      break;
    } else {
      debugSensorTestAttempt(sensorIndex, i + 1, false, temp);
    }
  }

  if (!success) {
    sensorConnected[sensorIndex] = false;
    lastReadSuccessful[sensorIndex] = false;
    debugSensorTestFailure(sensorIndex);
  }
}

/* -------------------------- LORAWAN HELPERS -------------------------- */
float getTemperatureToSend(int sensorIndex) {
  if (sensorConnected[sensorIndex] && currentTemperature[sensorIndex] != TEMP_ERROR_VALUE) {
    return currentTemperature[sensorIndex];
  } else if (lastValidTemperature[sensorIndex] != TEMP_ERROR_VALUE) {
    debugUsingLastValid(sensorIndex);
    return lastValidTemperature[sensorIndex];
  } else {
    debugSendingErrorValue(sensorIndex);
    return TEMP_ERROR_VALUE;
  }
}

/* -------------------------- DYNAMIC DUTY CYCLE -------------------------- */
uint32_t calculateDynamicDutyCycle(float temp) {
  if (temp <= TEMP_THRESHOLD_LOW) {
    return SLEEP_TIME_LOW_MS;
  } else if (temp >= TEMP_THRESHOLD_HIGH) {
    return SLEEP_TIME_HIGH_MS;
  } else {
    // Linear interpolation: y = mx + b
    // slope m = (y2 - y1) / (x2 - x1)
    float slope = (float)(SLEEP_TIME_HIGH_MS - SLEEP_TIME_LOW_MS) / (TEMP_THRESHOLD_HIGH - TEMP_THRESHOLD_LOW);
    float sleepTime = slope * (temp - TEMP_THRESHOLD_LOW) + SLEEP_TIME_LOW_MS;
    return (uint32_t)sleepTime;
  }
}

String getLoRaWANStatusString() {
  switch (deviceState) {
    case DEVICE_STATE_INIT: return "Initializing";
    case DEVICE_STATE_JOIN: return "Joining Network";
    case DEVICE_STATE_SEND: return "Sending Data";
    case DEVICE_STATE_CYCLE: return "Waiting";
    case DEVICE_STATE_SLEEP: return "Sleeping";
    default: return "Unknown State";
  }
}

static void prepareTxFrame(uint8_t port, float temperature0, float temperature1) {
  union {
    struct {
      float temperature0;         // MAX6675
      float temperature1;         // BMP280 temperature
      uint16_t batteryVoltage;    // mV
      uint8_t batteryPercentage;  // 0-100
      uint8_t sensorStatus;       // bit flags
    } data;
    uint8_t bytes[12];
  } payload;

  payload.data.temperature0 = temperature0;
  payload.data.temperature1 = temperature1;
  payload.data.batteryVoltage = batteryVoltage;
  payload.data.batteryPercentage = batteryPercentage;

  payload.data.sensorStatus = 0;
  if (sensorConnected[0]) payload.data.sensorStatus |= 0x01;
  if (lastReadSuccessful[0]) payload.data.sensorStatus |= 0x02;
  if (sensorConnected[1]) payload.data.sensorStatus |= 0x04;
  if (lastReadSuccessful[1]) payload.data.sensorStatus |= 0x08;
  if (batteryLow) payload.data.sensorStatus |= 0x10;
  if (successfulReadings[0] > failedReadings[0]) payload.data.sensorStatus |= 0x20;
  if (successfulReadings[1] > failedReadings[1]) payload.data.sensorStatus |= 0x40;
  if (batteryCharging) payload.data.sensorStatus |= 0x80;

  appDataSize = 12;
  for (int i = 0; i < appDataSize; i++) {
    appData[i] = payload.bytes[i];
  }

  debugLoRaPayload(temperature0, temperature1, batteryVoltage, batteryPercentage, payload.data.sensorStatus);
}

void handleLoRaWANStateMachine() {
  switch (deviceState) {
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

        if (!measurementsTakenThisCycle) {
          readBatteryVoltage();
          readTemperatures();
          measurementsTakenThisCycle = true;
        }

        float t0 = getTemperatureToSend(0);
        float t1 = getTemperatureToSend(1);  // BMP280 temp as second reading
        debugLoRaSendDetails(t0, t1);

        // --- DYNAMIC SLEEP CALCULATION ---
        float maxTemp = -999.0;
        bool validTempFound = false;

        // Find max valid temperature
        if (t0 != TEMP_ERROR_VALUE) {
          maxTemp = t0;
          validTempFound = true;
        }
        if (t1 != TEMP_ERROR_VALUE) {
          if (!validTempFound || t1 > maxTemp) {
            maxTemp = t1;
            validTempFound = true;
          }
        }

        if (validTempFound) {
          appTxDutyCycle = calculateDynamicDutyCycle(maxTemp);
          D_print("Dynamic Sleep: Max Temp ");
          D_print(maxTemp);
          D_print("C -> Interval ");
          D_print(appTxDutyCycle);
          D_println(" ms");
        } else {
          // If no valid sensors, default to slow (send current status of wrong sensors)
          appTxDutyCycle = SLEEP_TIME_HIGH_MS;
          D_println("Dynamic Sleep: No valid sensors. Defaulting to slow interval.");
        }
        // --------------------------------

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
void debugPrintStartupBanner() {
  D_println();
  D_println(String('=', 75));
  D_println("  MAX6675 + BMP280 Temperature Sensors + Battery + LoRaWAN");
  D_println("  First Boot (Cold Start)");
  D_println("  Libraries: GyverMAX6675, Adafruit_BMP280 | Board: Heltec ESP32 V3");
  D_println(String('=', 75));
}

void debugPrintResumeBanner() {
  D_println();
  D_println(String('-', 55));
  D_println("  Wake from Deep Sleep (Warm Start)");
  D_print("  Wake Counter: ");
  D_println(wakeCounter + 1);
  D_println(String('-', 55));
}

void debugPrintPinConfiguration() {
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

void debugSystemInitialized(bool first) {
  if (first) {
    D_println("System initialized (first run). Running diagnostics...");
  } else {
    D_println("System initialized (resume). Skipping full diagnostics.");
  }
}

void debugBatteryRead(uint16_t raw, float floatVoltage) {
  D_print("[readBatteryVoltage] ADC(avg): ");
  D_println(raw);
  D_print("[readBatteryVoltage] Float: ");
  D_println(floatVoltage, 3);
  D_print("[readBatteryVoltage] milliVolts: ");
  D_println(batteryVoltage);
  D_print("[readBatteryVoltage] Percentage: ");
  D_print(batteryPercentage);
  D_print("%");
  if (batteryLow) D_print(" - LOW BATTERY WARNING!");
  if (batteryCharging) D_print(" - CHARGING");
  D_println();
}

void debugBatteryInfo() {
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

void debugReadTempHeader() {
  D_println("=== Reading MAX6675 (ch0) + BMP280 (ch1) ===");
}
void debugReadTempFooter() {
  D_println("============================================");
}

void debugSingleTempResult(int sensorIndex, bool success, float tempC) {
  if (success) {
    D_print("Sensor ");
    D_print(sensorIndex);
    D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
    D_print(": SUCCESS - ");
    D_print(tempC, 2);
    D_println("°C");
  } else {
    D_print("Sensor ");
    D_print(sensorIndex);
    D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
    D_println(": FAILED - Sensor error or disconnected");
  }
}

void debugInvalidTemp(int sensorIndex, float temp, const String& reason) {
  D_print("Sensor ");
  D_print(sensorIndex);
  D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
  D_print(" INVALID ");
  D_print(temp, 2);
  D_print("°C - ");
  D_println(reason);
}

void debugSensorError(int sensorIndex, const String& err) {
  D_print("ERROR: Sensor ");
  D_print(sensorIndex);
  D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
  D_print(" - ");
  D_println(err);
}

void debugSensorTestStart() {
  D_println("Testing sensor connections...");
}

void debugSensorTestAttempt(int sensorIndex, int attempt, bool success, float temp) {
  D_print("Sensor ");
  D_print(sensorIndex);
  D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
  D_print(" Test ");
  D_print(attempt);
  D_print("/3: ");
  if (success) {
    D_print("SUCCESS - Temperature: ");
    D_print(temp, 1);
    D_println("°C");
  } else {
    D_println("FAILED");
  }
}

void debugSensorTestFailure(int sensorIndex) {
  if (sensorIndex == 0) {
    D_println("⚠ WARNING: Sensor 0 [MAX6675] test failed!");
    D_println("Check wiring:");
    D_println("  - VCC (3.3V)");
    D_println("  - GND");
    D_println("  - SCK GPIO " + String(clkPins[0]));
    D_println("  - SO  GPIO " + String(dataPins[0]));
    D_println("  - CS  GPIO " + String(csPins[0]));
    D_println("  - Thermocouple seating");
  } else {
    D_println("⚠ WARNING: Sensor 1 [BMP280] test failed!");
    D_println("Check I2C wiring:");
    D_println("  - 3V3 and GND");
    D_println("  - SDA / SCL (board defaults)");
    D_println("  - Address 0x76 or 0x77");
  }
}

void debugSensorTestSummary() {
  D_println();
  D_println("--- Sensor Test Summary ---");
  for (int i = 0; i < NUM_SENSORS; i++) {
    D_print("Sensor ");
    D_print(i);
    D_print(i == 0 ? " [MAX6675]" : " [BMP280]");
    D_print(": ");
    D_println(sensorConnected[i] ? "✓ WORKING" : "✗ FAILED");
  }
  D_println("---------------------------");
}

void debugLoRaSendHeader() {
  D_println();
  D_println(String('*', 60));
  D_println("PREPARING LORAWAN TRANSMISSION");
  D_println(String('*', 60));
}

void debugLoRaSendDetails(float t0, float t1) {
  D_println("MAX6675 (ch0): " + String(t0, 1) + "°C");
  D_println("BMP280  (ch1): " + String(t1, 1) + "°C");
  D_println("Battery: " + String(batteryVoltage / 1000.0, 2) + "V (" + String(batteryPercentage) + "%)");
  D_println("Charging: " + String(batteryCharging ? "Yes" : "No"));
  D_println("Wake Count: " + String(wakeCounter));
}

void debugLoRaSendFooter() {
  D_println("LoRaWAN transmission queued/completed");
  D_println("Next Sleep Time: " + String(appTxDutyCycle) + " ms");
  D_println(String('*', 60));
  D_println();
}

void debugLoRaPayload(float t0, float t1, uint16_t battMv, uint8_t battPct, uint8_t statusByte) {
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
  if (statusByte < 0x10) D_print("0");
  D_println(String(statusByte, HEX));
  D_print("  Payload bytes: [");
  for (int i = 0; i < appDataSize; i++) {
    if (i > 0) D_print(", ");
    D_print("0x");
    if (appData[i] < 0x10) D_print("0");
    D_print(String(appData[i], HEX));
  }
  D_println("]");
}

void debugUsingLastValid(int sensorIndex) {
  D_print("Sensor ");
  D_print(sensorIndex);
  D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
  D_println(" using last valid reading");
}

void debugSendingErrorValue(int sensorIndex) {
  D_print("Sensor ");
  D_print(sensorIndex);
  D_print(sensorIndex == 0 ? " [MAX6675]" : " [BMP280]");
  D_println(" sending error value");
}

void debugLoRaStatus(const String& state) {
  D_println("LORAWAN STATUS: " + state);
}

/* Required for LoRaWAN library persistence included above */
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda, 0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef, 0x67 };
uint32_t devAddr = (uint32_t)0x007e6ae1;