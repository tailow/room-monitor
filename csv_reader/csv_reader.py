import serial
import csv
from datetime import datetime

# === Configuration ===
PORT = "/dev/ttyUSB0"  # or "COM3" on Windows
BAUD = 115200
CSV_FILE = "sensor_log.csv"

# === Open serial connection ===
ser = serial.Serial(PORT, BAUD, timeout=2)

# === Create CSV header if not exists ===
with open(CSV_FILE, "a", newline="") as f:
    writer = csv.writer(f)
    if f.tell() == 0:  # file is empty
        writer.writerow(["timestamp", "temp1", "temp2", "humidity"])

print(f"Logging serial data from {PORT} to {CSV_FILE}...")

while True:
    try:
        line = ser.readline().decode(errors="ignore").strip()
        if not line:
            continue

        # Expected format: T1:23.45,T2:24.01,H:46.7
        parts = line.split(",")
        data = {}

        for p in parts:
            if ":" in p:
                key, val = p.split(":")
                data[key.strip()] = float(val.strip())

        # Only log if we have all fields
        if all(k in data for k in ("T1", "T2", "H")):
            timestamp = datetime.now().isoformat(timespec="milliseconds")

            row = [timestamp, data["T1"], data["T2"], data["H"]]

            print(row)

            with open(CSV_FILE, "a", newline="") as f:
                csv.writer(f).writerow(row)

    except KeyboardInterrupt:
        print("\nStopped by user.")
        break
    except Exception as e:
        print("Error:", e)
