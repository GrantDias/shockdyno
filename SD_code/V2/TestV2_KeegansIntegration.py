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
        self._lock = threading.Lock()
        self._writer = None
        self._file = None
        self.last_millis = ''  # holds the last millis value seen
        self.prev_time = None  # previous timestamp in seconds
        self.prev_pos = None   # previous position (linear pot)

    def start(self):
        """Start the logging thread and open the CSV file."""
        self._file = open(self.csv_path, 'w', newline='')
        self._writer = csv.writer(self._file)
        # Add Velocity as 4th column
        self._writer.writerow(['Millis', 'Load_Cell', 'Linear_Pot (scaled)', 'Velocity', 'Thermocouple'])
        self._file.flush()

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        print(f" Logging to {self.csv_path}")

    def stop(self):
        """Stop logging and close the CSV file."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._file:
            self._file.close()

    def log_message(self, message):
        """Logs a message aligned with the last known millis value."""
        with self._lock:
            self._writer.writerow([self.last_millis, message, '', '', ''])
            self._file.flush()

    def _run(self):
        """Thread function: continuously read from serial and log data with velocity."""
        while not self._stop_event.is_set():
            try:
                # Wait for sync byte 0xAA
                if self.ser.read(1) != b'\xAA':
                    continue

                # Read line of CSV-style data from Arduino
                raw = self.ser.readline()
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line or ',' not in line:
                    continue

                parts = [p.strip() for p in line.split(',')]
                if len(parts) < 3:
                    continue  # expects at least millis, load cell, lin pot

                # Parse millis and linear potentiometer
                try:
                    current_time = float(parts[0]) / 1000  # convert ms to seconds
                    lin_pot_val = float(parts[2]) / 1023 * 7.43  # scale
                    parts[2] = f"{lin_pot_val:.3f}"
                except ValueError:
                    lin_pot_val = float('nan')
                    current_time = None
                    parts[2] = "NaN"

                # Compute velocity (derivative)
                if self.prev_time is not None and current_time is not None:
                    dt = current_time - self.prev_time
                    if dt > 0:
                        velocity = (lin_pot_val - self.prev_pos) / dt
                    else:
                        velocity = 0
                else:
                    velocity = 0  # first data point

                # Save current values for next iteration
                self.prev_time = current_time
                self.prev_pos = lin_pot_val

                # Append velocity as 4th column
                parts.insert(3, f"{velocity:.3f}")  # insert before thermocouple

                # Save millis for alignment with switch messages
                self.last_millis = parts[0]

                print(parts)  # Console mirror
                with self._lock:
                    self._writer.writerow(parts)
                    self._file.flush()

            except serial.SerialException as e:
                print(f"Serial error: {e}")
                break


def open_serial(port, baud):
    """Open the serial port safely."""
    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)  # give Arduino time to reset
        print(f" Connected to {port} at {baud} baud.")
        return ser
    except serial.SerialException as e:
        print(f" Error opening {port}: {e}")
        sys.exit(1)


def run_interactive(ser, logger):
    """Interactive loop for sending duty cycle commands."""
    prev_key = None
    try:
        while True:
            user_input = input().strip()
            if user_input.lower() in {'q', 'quit', 'exit'}:
                break
            if user_input and user_input[0].isdigit():
                key = user_input[0]
                ser.write(key.encode('ascii'))
                ser.flush()
                print(f"  Sent duty '{key}'")

                if prev_key is not None and key != prev_key:
                    msg = f"Switched from {prev_key} to {key}"
                    logger.log_message(msg)
                    print(f" {msg}")

                prev_key = key
    except KeyboardInterrupt:
        print("\n Exiting...")
    finally:
        ser.write(b'0')
        ser.flush()
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
    print(f" Data saved to {csv_name}")


if __name__ == "__main__":
    main()
