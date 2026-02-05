import serial
import serial.tools.list_ports
import pandas as pd
import time
import sys

# --- CONFIGURATION ---
EXCEL_FILE = "devices.xlsx"
SHEET_NAME = "Devices"
BAUD_RATE = 115200
TIMEOUT = 2  # Seconds to wait for serial connection


def get_serial_ports():
    """Lists available serial ports."""
    return list(serial.tools.list_ports.comports())


def load_and_filter_data():
    """Loads Excel, sanitizes headers, and filters invalid rows."""
    try:
        # Load the sheet
        df = pd.read_excel(EXCEL_FILE, sheet_name=SHEET_NAME, engine="openpyxl")

        # Strip whitespace from column names
        df.columns = df.columns.str.strip()

        # Check for required columns before filtering
        required_cols = [
            "SERIAL_NUMBER",
            "DEV_EUI",
            "APP_KEY",
            "VOLTAGE_MEASURED",
            "VOLTAGE_REPORTED",
        ]
        missing = [col for col in required_cols if col not in df.columns]
        if missing:
            print(f"Error: Missing columns in Excel: {missing}")
            return None

        # FILTER: Keep only rows where SERIAL_NUMBER and DEV_EUI exist
        df_clean = df.dropna(subset=["SERIAL_NUMBER", "DEV_EUI", "APP_KEY"])

        if df_clean.empty:
            print(
                "Error: No valid devices found (Check SERIAL_NUMBER and DEV_EUI columns)."
            )
            return None

        return df_clean

    except FileNotFoundError:
        print(f"Error: '{EXCEL_FILE}' not found.")
        return None
    except Exception as e:
        print(f"Error reading Excel: {e}")
        return None


def main():
    print("--- ESP32 LoRaWAN Provisioner ---\n")

    # 1. Load & Filter Excel Data
    df = load_and_filter_data()
    if df is None:
        return

    # 2. Select Serial Port
    ports = get_serial_ports()
    if not ports:
        print("No serial ports found! Check your USB connection.")
        return

    print("Available Serial Ports:")
    for i, p in enumerate(ports):
        print(f" [{i}] {p.device} ({p.description})")

    try:
        idx = int(input("\nSelect Port Index (e.g., 0): "))
        selected_port = ports[idx].device
    except (ValueError, IndexError):
        print("Invalid selection.")
        return

    # 3. Select Device (Row)
    print(f"\nScanning '{EXCEL_FILE}' for valid devices...")

    # Display preview of VALID devices only
    preview_cols = ["SERIAL_NUMBER", "DEV_EUI"]
    # Force integer formatting for Serial Number for cleaner display
    print(df[preview_cols].astype({"SERIAL_NUMBER": "int"}).to_string(index=False))
    print("...")

    try:
        target_sn = int(input("\nEnter SERIAL_NUMBER to provision: "))
    except ValueError:
        print("Invalid input. Serial Number must be a number.")
        return

    # Find the row with the matching SERIAL_NUMBER
    device_row = df[df["SERIAL_NUMBER"] == target_sn]

    if device_row.empty:
        print(f"Error: Serial Number {target_sn} not found in valid list.")
        return

    # 4. Extract and Clean Data
    dev_eui = str(device_row.iloc[0]["DEV_EUI"]).strip()
    app_key = str(device_row.iloc[0]["APP_KEY"]).strip()

    # Handle calibration data (fill NaN with 1.0)
    v_meas = device_row.iloc[0]["VOLTAGE_MEASURED"]
    v_rep = device_row.iloc[0]["VOLTAGE_REPORTED"]

    # If cell is empty (NaN), default to 1.0
    if pd.isna(v_meas):
        v_meas = 1.0
    if pd.isna(v_rep):
        v_rep = 1.0

    # 5. Send Data
    # Format: DEV_EUI,APP_KEY,VOLTAGE_MEASURED,VOLTAGE_REPORTED
    payload = f"{dev_eui},{app_key},{v_meas},{v_rep}\n"

    print(f"\n--- Provisioning Device {target_sn} ---")
    print(f"Target: {selected_port}")
    print(f"Payload: {payload.strip()}")

    try:
        with serial.Serial(selected_port, BAUD_RATE, timeout=TIMEOUT) as ser:
            ser.dtr = False
            ser.rts = False

            print("Opening port... waiting 2s for boot...")
            time.sleep(2)

            ser.reset_input_buffer()

            print("Sending data...")
            ser.write(payload.encode("utf-8"))

            time.sleep(1)

            if ser.in_waiting:
                response = ser.read(ser.in_waiting).decode("utf-8", errors="replace")
                print(f"ESP32 Response:\n{response}")
            else:
                print("No response received from ESP32.")

            print("\n--- Done ---")

    except serial.SerialException as e:
        print(f"Serial Error: {e}")


if __name__ == "__main__":
    main()
