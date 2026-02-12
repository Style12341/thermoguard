/*
 * Thermoguard V3 - RUI3 Port (RAK3172)
 * * MIGRATION NOTES:
 * 1. Sleep: RUI3 resumes context. No RTC_DATA_ATTR needed.
 * 2. Storage: Credentials stored in RUI3 Internal Config; Calibration stored in User Flash.
 * 3. Pinout: CHECK ALL PIN DEFINITIONS below for your specific hardware wiring.
 */

#include "Arduino.h"
#include <GyverMAX6675.h>   // Ensure this library is compatible or use MAX6675 library
#include <Adafruit_BMP280.h>
#include <Wire.h>

/* -------------------------- DEBUG MACROS -------------------------- */
#define DEBUG

#ifdef DEBUG
  #define D_SerialBegin(...) Serial.begin(__VA_ARGS__)
  #define D_print(...)       Serial.print(__VA_ARGS__)
  #define D_println(...)     Serial.println(__VA_ARGS__)
  #define D_printf(...)      Serial.printf(__VA_ARGS__)
#else
  #define D_SerialBegin(...)
  #define D_print(...)
  #define D_println(...)
  #define D_printf(...)
#endif

/* -------------------------- PIN DEFINITIONS (VERIFY THESE!) -------------------------- */
/* RAK3178 / STM32WLE5 Pin Mapping - ADAPT TO YOUR BOARD */
#define CLK_PIN_1        WB_IO1   // Example: Software SPI SCK for MAX6675
#define DATA_PIN_1       WB_IO2   // Example: Software SPI MISO for MAX6675
#define CS_PIN_1         WB_IO3   // Example: CS for MAX6675

// Battery Reading
// RAK modules often use a dedicated internal divider. If using external:
#define VBAT_READ_PIN    WB_A0    // Analog Input Pin
#define ADC_CTRL_PIN     WB_IO4   // Pin to turn on voltage divider (if exists)
#define BATTERY_CHARGING_PIN WB_IO5 

#define ADC_RESOLUTION      12
#define ADC_MAX_VOLTAGE     3.3   // RAK/STM32 usually 3.3V or 3.6V logic
#define VOLTAGE_R1          390   // External Divider
#define VOLTAGE_R2          100   // External Divider

#define TEMP_ERROR_VALUE    -999.0
#define NUM_SENSORS         2
#define FLASH_CALIBRATION_ADDR 0x00 // Offset start for User Flash

/* -------------------------- DYNAMIC SLEEP CONFIG -------------------------- */
#define TEMP_THRESHOLD_LOW  20.0f
#define TEMP_THRESHOLD_HIGH 100.0f
#define SLEEP_TIME_LOW_MS   90000  // 1.5 min
#define SLEEP_TIME_HIGH_MS  25000  // 25 sec

/* -------------------------- SENSOR OBJECTS -------------------------- */
// GyverMAX6675 allows software SPI on any pins
GyverMAX6675<CLK_PIN_1, DATA_PIN_1, CS_PIN_1> tempSensor0;
Adafruit_BMP280 bmp;

/* -------------------------- GLOBAL VARS -------------------------- */
// In RUI3, these retained their value during sleep (RAM retention)
float currentTemperature[NUM_SENSORS] = {0.0, 0.0};
float lastValidTemperature[NUM_SENSORS] = {TEMP_ERROR_VALUE, TEMP_ERROR_VALUE};
unsigned long successfulReadings[NUM_SENSORS] = {0, 0};
unsigned long failedReadings[NUM_SENSORS] = {0, 0};
bool sensorConnected[NUM_SENSORS] = {false, false};
bool lastReadSuccessful[NUM_SENSORS] = {false, false};

uint16_t batteryVoltage = 0;
uint8_t batteryPercentage = 0;
bool batteryLow = false;
bool batteryCharging = false;
bool bmpInitialized = false;

// Calibration Data (Loaded from Flash)
struct CalibrationData {
  float vMeas;
  float vRep;
  uint32_t magic; // To detect if flash is empty
};
CalibrationData calData = {1.0, 1.0, 0};

/* -------------------------- LORAWAN CONFIG -------------------------- */
#define LORAWAN_APP_PORT 2
// OTAA Keys will be loaded via Provisioning or existing API settings

/* -------------------------- FORWARD DECLARATIONS -------------------------- */
void runProvisioningMode();
void readTemperatures();
void readBatteryVoltage();
void prepareTxFrame(uint8_t port, float t0, float t1);
void saveCalibration();
void loadCalibration();

/* -------------------------- SETUP -------------------------- */
void setup()
{
    // 1. Init Serial
    Serial.begin(115200, RAK_AT_MODE);
    delay(2000); // Give time for USB Serial
    
    D_println("==========================================");
    D_println("   RAK3178 Thermoguard Port (RUI3)");
    D_println("==========================================");

    // 2. Hardware Init
    pinMode(ADC_CTRL_PIN, OUTPUT);
    pinMode(BATTERY_CHARGING_PIN, INPUT); 
    // Analog resolution default is usually 10 or 12 depending on core, setting explicitly:
    analogReadResolution(ADC_RESOLUTION);

    // 3. Load Calibration from User Flash
    loadCalibration();

    // 4. Check Provisioning State
    // We check if DevEUI is all zeros or empty. 
    uint8_t checkEui[8];
    api.lorawan.deui.get(checkEui, 8);
    bool isZero = true;
    for(int i=0; i<8; i++) if(checkEui[i] != 0) isZero = false;

    if (isZero) {
        D_println("[Setup] No Credentials found (DevEUI is 0). Enter Provisioning Mode.");
        runProvisioningMode();
    } else {
        D_println("[Setup] Credentials found. Proceeding.");
    }

    // 5. Initialize Sensors
    // BMP280
    Wire.begin(); // Uses default SDA/SCL pins of the module
    if (bmp.begin(0x76) || bmp.begin(0x77)) {
        bmpInitialized = true;
        bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                        Adafruit_BMP280::SAMPLING_X1,
                        Adafruit_BMP280::SAMPLING_X1,
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_500);
        sensorConnected[1] = true;
    } else {
        D_println("[Setup] BMP280 Not Found");
        sensorConnected[1] = false;
    }
    
    // MAX6675 is software driven, no init needed beyond object creation
    sensorConnected[0] = true; // Assume present for now

    // 6. Setup LoRaWAN
    if(api.lorawan.nwm.get() != 1) {
        api.lorawan.nwm.set(); // Set to LoRaWAN mode
        api.system.reboot();
    }
    
    // Configure LoRaWAN Settings
    api.lorawan.band.set(RAK_REGION_AU915); // Match your region
    api.lorawan.deviceClass.set(RAK_LORA_CLASS_A);
    api.lorawan.njm.set(RAK_LORA_OTAA);
    api.lorawan.adr.set(true);
    api.lorawan.rety.set(1);
    api.lorawan.cfm.set(1); // Confirmed messages
    
    // Start Join
    D_print("[LoRa] Joining...");
    if (!api.lorawan.join()) {
        D_println("Join Request Failed");
    }

    // Register Callbacks
    api.lorawan.registerRecvCallback(recvCallback);
    api.lorawan.registerJoinCallback(joinCallback);
    api.lorawan.registerSendCallback(sendCallback);
}

/* -------------------------- CALLBACKS -------------------------- */
void joinCallback(int32_t status) {
    if (status == RAK_LORAMAC_STATUS_OK) {
        D_println("[Callback] LoRaWAN Joined Successfully");
    } else {
        D_printf("[Callback] Join Failed: %d\r\n", status);
    }
}

void sendCallback(int32_t status) {
    if (status == RAK_LORAMAC_STATUS_OK) {
        D_println("[Callback] Send Confirmed/OK");
    } else {
        D_println("[Callback] Send Failed");
    }
}

void recvCallback(SERVICE_LORA_RECEIVE_T * data) {
    D_println("[Callback] Downlink Received");
}

/* -------------------------- LOOP -------------------------- */
void loop()
{
    // 1. Wait for Join if not joined
    if (api.lorawan.njs.get() == 0) {
        // Not joined yet. 
        // RUI3 handles retries in background usually, but we can prevent app logic
        D_print(".");
        delay(5000);
        return; 
    }

    // 2. Read Data
    readBatteryVoltage();
    readTemperatures();

    // 3. Prepare Payload
    float t0 = (sensorConnected[0] && lastReadSuccessful[0]) ? currentTemperature[0] : TEMP_ERROR_VALUE;
    float t1 = (sensorConnected[1] && lastReadSuccessful[1]) ? currentTemperature[1] : TEMP_ERROR_VALUE;
    
    // Fallback logic
    if (t0 == TEMP_ERROR_VALUE && lastValidTemperature[0] != TEMP_ERROR_VALUE) t0 = lastValidTemperature[0];
    if (t1 == TEMP_ERROR_VALUE && lastValidTemperature[1] != TEMP_ERROR_VALUE) t1 = lastValidTemperature[1];

    prepareTxFrame(LORAWAN_APP_PORT, t0, t1);

    // 4. Calculate Dynamic Sleep
    float maxTemp = -999.0;
    if (t0 > maxTemp) maxTemp = t0;
    if (t1 > maxTemp) maxTemp = t1;

    uint32_t sleepTime = SLEEP_TIME_HIGH_MS;
    if (maxTemp > -100) { // Valid temp found
        if (maxTemp <= TEMP_THRESHOLD_LOW) sleepTime = SLEEP_TIME_LOW_MS;
        else if (maxTemp >= TEMP_THRESHOLD_HIGH) sleepTime = SLEEP_TIME_HIGH_MS;
        else {
            float slope = (float)(SLEEP_TIME_HIGH_MS - SLEEP_TIME_LOW_MS) / (TEMP_THRESHOLD_HIGH - TEMP_THRESHOLD_LOW);
            sleepTime = slope * (maxTemp - TEMP_THRESHOLD_LOW) + SLEEP_TIME_LOW_MS;
        }
    }
    
    D_printf("[Loop] Max Temp: %.2f C -> Sleep Time: %u ms\r\n", maxTemp, sleepTime);
    
    // 5. Send Packet
    // (Buffer is populated in prepareTxFrame into global 'collected_data' or similar, 
    // but here we pass the buffer directly).
    // Note: RUI3 send is non-blocking. The sleep below needs to handle this.
    // RUI3 `sleep.all` automatically waits for TX to finish or schedules wakeup.
    
    // 6. Sleep
    // RUI3 specific: sleep.all puts the device in low power and sets a timer.
    // When timer expires, code resumes HERE (next loop iteration).
    api.system.sleep.all(sleepTime); 
}

/* -------------------------- HELPER FUNCTIONS -------------------------- */

void readTemperatures() {
    // Sensor 0: MAX6675
    if (tempSensor0.readTemp()) {
        float temp = tempSensor0.getTemp();
        if (temp > -50 && temp < 1000) {
            currentTemperature[0] = temp;
            lastValidTemperature[0] = temp;
            lastReadSuccessful[0] = true;
            successfulReadings[0]++;
            D_printf("MAX6675: %.2f C\r\n", temp);
        } else {
            lastReadSuccessful[0] = false;
            failedReadings[0]++;
        }
    } else {
        lastReadSuccessful[0] = false;
        failedReadings[0]++;
        D_println("MAX6675 Read Failed");
    }

    // Sensor 1: BMP280
    if (bmpInitialized && bmp.takeForcedMeasurement()) {
         float temp = bmp.readTemperature();
         if (!isnan(temp) && temp > -50 && temp < 1000) {
             currentTemperature[1] = temp;
             lastValidTemperature[1] = temp;
             lastReadSuccessful[1] = true;
             successfulReadings[1]++;
             D_printf("BMP280: %.2f C\r\n", temp);
         } else {
             lastReadSuccessful[1] = false;
             failedReadings[1]++;
         }
    } else if (bmpInitialized) {
        lastReadSuccessful[1] = false;
        failedReadings[1]++;
        D_println("BMP280 Read Failed");
    }
}

void readBatteryVoltage() {
    digitalWrite(ADC_CTRL_PIN, HIGH);
    delay(10); // Allow stabilize
    
    // RUI3 API for analog read
    // Note: If using internal battery pin of RAK3172/8, use api.system.bat.get()
    // But respecting your custom divider logic:
    uint32_t rawSum = 0;
    for(int i=0; i<3; i++) {
        rawSum += analogRead(VBAT_READ_PIN);
    }
    digitalWrite(ADC_CTRL_PIN, LOW);
    
    float avgRaw = rawSum / 3.0;
    
    // Calc Logic
    // V = Raw * (3.3 / 4096) * ((R1+R2)/R2) * Calibration
    const float adcMax = (1 << ADC_RESOLUTION) - 1;
    float factor = (ADC_MAX_VOLTAGE / adcMax) * ((float)(VOLTAGE_R1 + VOLTAGE_R2) / (float)VOLTAGE_R2) * (calData.vMeas / calData.vRep);
    
    float voltage = avgRaw * factor;
    batteryVoltage = (uint16_t)(voltage * 1000); // mV
    
    // Percentage
    if (batteryVoltage >= 4200) batteryPercentage = 100;
    else if (batteryVoltage <= 3200) batteryPercentage = 0;
    else batteryPercentage = (uint8_t)((batteryVoltage - 3200) * 100 / (4200 - 3200));
    
    batteryCharging = digitalRead(BATTERY_CHARGING_PIN);
    batteryLow = (batteryVoltage < 3400);
    
    D_printf("Batt: %d mV (%d%%)\r\n", batteryVoltage, batteryPercentage);
}

void prepareTxFrame(uint8_t port, float t0, float t1) {
    uint8_t payload[12];
    
    // Same struct packing as your ESP32 code
    union {
        struct {
            float temperature0;
            float temperature1;
            uint16_t batteryVoltage;
            uint8_t batteryPercentage;
            uint8_t sensorStatus;
        } data;
        uint8_t bytes[12];
    } pack;

    pack.data.temperature0 = t0;
    pack.data.temperature1 = t1;
    pack.data.batteryVoltage = batteryVoltage;
    pack.data.batteryPercentage = batteryPercentage;
    
    // Status Byte Construction
    uint8_t status = 0;
    if (sensorConnected[0]) status |= 0x01;
    if (lastReadSuccessful[0]) status |= 0x02;
    if (sensorConnected[1]) status |= 0x04;
    if (lastReadSuccessful[1]) status |= 0x08;
    if (batteryLow) status |= 0x10;
    if (successfulReadings[0] > failedReadings[0]) status |= 0x20;
    if (successfulReadings[1] > failedReadings[1]) status |= 0x40;
    if (batteryCharging) status |= 0x80;
    pack.data.sensorStatus = status;

    if (api.lorawan.send(12, pack.bytes, port, true)) {
        D_println("Packet Queued");
    } else {
        D_println("Packet Queue Failed");
    }
}

/* -------------------------- PROVISIONING / STORAGE -------------------------- */

void loadCalibration() {
    uint8_t buff[sizeof(CalibrationData)];
    if (api.system.flash.get(FLASH_CALIBRATION_ADDR, buff, sizeof(CalibrationData))) {
        memcpy(&calData, buff, sizeof(CalibrationData));
        if (calData.magic != 0xA5A5A5A5) {
            // Defaults
            calData.vMeas = 1.0;
            calData.vRep = 1.0;
        }
    }
}

void saveCalibration() {
    calData.magic = 0xA5A5A5A5;
    api.system.flash.set(FLASH_CALIBRATION_ADDR, (uint8_t*)&calData, sizeof(CalibrationData));
}

void hexStringToBytes(String hexStr, uint8_t *output, size_t len) {
    hexStr.replace(" ", "");
    hexStr.replace("0x", "");
    for (size_t i = 0; i < len; i++) {
        char buffer[3] = {0};
        if (i*2 + 1 < hexStr.length()) {
            buffer[0] = hexStr[i*2];
            buffer[1] = hexStr[i*2+1];
            output[i] = (uint8_t)strtol(buffer, NULL, 16);
        }
    }
}

void runProvisioningMode() {
    readBatteryVoltage();
    Serial.println("--- PROVISIONING REQUIRED ---");
    Serial.printf("PRE-PROV VOLTAGE: %.3f V\r\n", batteryVoltage/1000.0);
    Serial.println("Send: DEVEUI,APPKEY,V_MEAS,V_REP");
    
    while(true) {
        if (Serial.available()) {
            String line = Serial.readStringUntil('\n');
            line.trim();
            if (line.length() > 20) {
                // Simplified CSV parsing
                int c1 = line.indexOf(',');
                int c2 = line.indexOf(',', c1+1);
                int c3 = line.indexOf(',', c2+1);
                
                if (c1 > 0 && c2 > 0) {
                    String sDevEui = line.substring(0, c1);
                    String sAppKey = line.substring(c1+1, c2);
                    String sVMeas = (c3 > 0) ? line.substring(c2+1, c3) : line.substring(c2+1);
                    String sVRep = (c3 > 0) ? line.substring(c3+1) : "1.0";
                    
                    uint8_t bufEui[8];
                    uint8_t bufKey[16];
                    hexStringToBytes(sDevEui, bufEui, 8);
                    hexStringToBytes(sAppKey, bufKey, 16);
                    
                    // Set RUI3 Credentials
                    api.lorawan.deui.set(bufEui, 8);
                    api.lorawan.appkey.set(bufKey, 16);
                    
                    // Save Calibration
                    calData.vMeas = sVMeas.toFloat();
                    calData.vRep = sVRep.toFloat();
                    saveCalibration();
                    
                    Serial.println("PROVISIONING DONE. REBOOTING...");
                    delay(1000);
                    api.system.reboot();
                }
            }
        }
        delay(100);
    }
}