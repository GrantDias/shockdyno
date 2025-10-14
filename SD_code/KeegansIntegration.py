import sys
import threading
import time
import csv
from datetime import datetime
from typing import Optional

try:
    import serial  # pyserial
except ImportError:
    print("Missing dependency: pyserial. Install with: pip install pyserial")
    sys.exit(1)


class SerialLogger:
    """Reads lines from a serial port and appends them to a CSV file.

    Expected Arduino line format: "millis,load_cell,linear_pot(optional)"
    Lines without commas are ignored.
    """

    def __init__(self, ser: serial.Serial, csv_path: str) -> None:
        self.ser = ser
        self.csv_path = csv_path
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="SerialLogger", daemon=True)
        self._thread.start()

    def stop(self, join_timeout: float = 2.0) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=join_timeout)

    def _run(self) -> None:
        # Open CSV and write headers once
        with open(self.csv_path, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Millis', 'Load_Cell', 'Linear_Pot'])
            print(f"Reading data and saving to {self.csv_path}")
            print("Press digits 0-9 to set motor duty, 'q' to quit.")

            while not self._stop_event.is_set():
                try:
                    if self.ser.in_waiting > 0:
                        raw = self.ser.readline()
                        try:
                            line = raw.decode('utf-8', errors='strict').rstrip()
                        except UnicodeDecodeError:
                            # Fallback decode to skip malformed bytes
                            line = raw.decode('utf-8', errors='ignore').rstrip()

                        if not line or ',' not in line:
                            continue

                        parts = [p.strip() for p in line.split(',')]
                        if len(parts) >= 3:
                            # Mirror to console for quick monitoring
                            print(f"Millis: {parts[0]}, Load: {parts[1]}, Pot: {parts[2]}")
                        writer.writerow(parts)
                        file.flush()
                    else:
                        time.sleep(0.01)
                except serial.SerialException as e:
                    print(f"Error reading from serial port: {e}")
                    break


def open_serial(port: str, baud: int, timeout: float = 1.0) -> serial.Serial:
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        # Give Arduino time to reset and start
        time.sleep(2)
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port {port}: {e}")
        sys.exit(1)


def send_duty_cycle(ser: serial.Serial, key: str) -> None:
    """Send a single ASCII digit '0'..'9' to the Arduino to set PWM."""
    if key < '0' or key > '9':
        return
    try:
        ser.write(key.encode('ascii'))
        ser.flush()
        print(f"Sent duty '{key}'")
    except serial.SerialException as e:
        print(f"Error writing to serial port: {e}")


def run_interactive(ser: serial.Serial, logger: SerialLogger) -> None:
    """Interactive loop: read keyboard input and send motor commands."""
    try:
        while True:
            user_input = input().strip()  # blocking, keeps CPU usage low
            if user_input.lower() in {'q', 'quit', 'exit', 'stop', 'terminate'}:
                print("Exiting on user request...")
                break
            if user_input and user_input[0].isdigit():
                send_duty_cycle(ser, user_input[0])
            else:
                print("Enter a digit 0-9 to set duty cycle, or 'q' to quit.")
    except KeyboardInterrupt:
        print("\nExiting (Ctrl+C)...")
    finally:
        send_duty_cycle(ser, '0') #turn motor off on exit
        logger.stop()


def main() -> None:
    # Defaults match your existing Python and Arduino sketch
    port = "COM8"  # INPUT YOUR PORT NAME FOR THE ARDUNIO STEPS TO FIND THIS: 1. Lookup device manager 2. Go to "ports (COM & LPT)" Input the name
    baud = 9600

    # Auto-named CSV file
    csv_name = f"Shock_Dyno_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

    ser = open_serial(port, baud)

    logger = SerialLogger(ser, csv_name)
    logger.start()

    # Optionally send a startup duty cycle (commented out)
    # send_duty_cycle(ser, '0')

    run_interactive(ser, logger)

    try:
        ser.close()
    except Exception:
        pass
    print(f"Data saved to {csv_name}")


if __name__ == "__main__":
    main()


