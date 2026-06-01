# ThermoGuard — Monorepo

Multi-tenant IoT temperature monitoring SaaS. Ingest sensor data via MQTT (LoRaWAN + direct MQTT), run rules engine, alert via Telegram, real-time dashboard with HACCP compliance reporting.

## Repo structure

```
Thermoguard/
  app/                  Elixir/Phoenix backend (git submodule)
  thermoguard_direct/   ESP32 direct MQTT N-sensor firmware (PlatformIO)
  thermoguard_v3/       LoRaWAN v3 dual-sensor firmware (Arduino)
  device_provisioner.py Python script for LoRaWAN device provisioning
  devices.xlsx          Device registry for provisioning
  Backups/              ChirpStack dumps, test data
```

### `app/` — Backend

Elixir/Phoenix LiveView + Vue 3 + TimescaleDB. Multi-tenant via Triplex (schema-per-tenant). MQTT ingestion, rules engine, Telegram alerts, compliance reports. Spanish-first UI.

**Remote:** `git@github.com:Style12341/thermoguard_ex.git` (private)

**Before any backend work:** `cd app && git pull origin dev`

The `app/` directory has its own `AGENTS.md` with full architecture docs, conventions, and skill registry.

### `thermoguard_direct/` — Direct MQTT Firmware

ESP32 (DOIT DevKit v1) + DS18B20 temperature sensors. PlatformIO project. Each DS18B20 sensor is its own "device" in ThermoGuard. Publishes directly to MQTT broker — no LoRaWAN gateway needed.

- WiFiManager config portal (MQTT broker, app_id, sensor count, read interval)
- ElegantOTA for wireless firmware updates
- Device identity = DS18B20 1-Wire address (16-char hex)
- MQTT topic: `thermoguard/direct/<app_id>/<sensor_address>/data`
- Payload: `{"version":"1.0","temp":23.5,"rssi":-65,"temp_ok":true}`

### `thermoguard_v3/` — LoRaWAN Firmware

LoRaWAN v3 dual-sensor device. Single `.ino` file (Arduino IDE). Uses MAX6675 thermocouple + BMP280. Connects via ChirpStack to ThermoGuard. Provisioned via serial using `device_provisioner.py`.

### `device_provisioner.py`

Reads `devices.xlsx` → connects to LoRaWAN device via serial → programs DevEUI and AppKey into NVS. Used for mass provisioning.

## Quick Reference

```bash
# Pull latest backend
cd app && git pull origin dev

# Run backend tests
cd app && mix test

# Direct firmware — build & upload
cd thermoguard_direct && pio run -t upload

# Direct firmware — serial monitor
cd thermoguard_direct && pio device monitor
```
