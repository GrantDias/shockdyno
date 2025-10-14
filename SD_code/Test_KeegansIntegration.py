import sys
import threading
import time
import csv
from datetime import datetime

try:
    import serial
except ImportError:
    print("Install pyserial: pip install pyserial")
    sys.exit(1)

class SerialLogger:
    def __init__(self, ser, csv_path):
        self.ser = ser
        self.csv_path = csv_path
        self._stop_event = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2)

    def _run(self):
        with open(self.csv_path, 'w', newline='') as file:
            writer = csv.writer(file)
            # Update headers when adding sensors
            writer.writerow(['Millis', 'Load_Cell', 'Linear_Pot', 'Thermocouple'])  

            print(f"Logging to {self.csv_path}")
            while not self._stop_event.is_set():
                try:
                    if self.ser.read(1) != b'\xAA':  # wait for sync
                        continue

                    raw = self.ser.readline()
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if not line or ',' not in line:
                        continue

                    parts = [p.strip() for p in line.split(',')]
                    if len(parts) < 1:  # adjust if adding sensors
                        continue

                    print(parts)  # console mirror
                    writer.writerow(parts)
                    file.flush()
                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                    break

def open_serial(port, baud):
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        return ser
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        sys.exit(1)

def send_duty_cycle(ser, key):
    if key.isdigit():
        ser.write(key.encode('ascii'))
        ser.flush()
        print(f"Sent duty '{key}'")

def run_interactive(ser, logger):
    try:
        while True:
            user_input = input().strip()
            if user_input.lower() in {'q', 'quit', 'exit'}:
                break
            if user_input:
                send_duty_cycle(ser, user_input[0])
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        send_duty_cycle(ser, '0')
        logger.stop()

def main():
    port = "COM8"   # update for your board
    baud = 115200
    csv_name = f"Shock_Dyno_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    ser = open_serial(port, baud)
    logger = SerialLogger(ser, csv_name)
    logger.start()
    run_interactive(ser, logger)

    ser.close()
    print(f"Data saved to {csv_name}")

if __name__ == "__main__":
    main()
