import serial
import time
import csv
from datetime import datetime

arduino_port = "COM8"  # Replace with your Arduino port
baud_rate = 9600       # Changed to match your Arduino code

try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    time.sleep(2)  # Wait for connection to establish
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
    exit()

# Auto-generate filename with timestamp (optional)
filename = f'Shock_Dyno_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'

with open(filename, 'w', newline='') as file:
    writer = csv.writer(file)
    
    # Optional: Add headers
    writer.writerow(['Millis', 'Load_Cell', 'Linear_Pot'])
    
    print(f'Reading data and saving to {filename}')
    print('Press Ctrl+C to stop...')
    
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').rstrip()
                if data and ',' in data:  # Only process valid data lines
                    parsed_data = data.split(',')
                    
                    # Optional: Print to console for monitoring
                    if len(parsed_data) >= 3:
                        print(f"Millis: {parsed_data[0]}, Load: {parsed_data[1]}, Pot: {parsed_data[2]}")
                    
                    # Write the parsed data into the CSV file
                    writer.writerow(parsed_data)
                    file.flush()  # Ensure data is written immediately

        except serial.SerialException as e:
            print(f"Error reading from serial port: {e}")
            break
        except KeyboardInterrupt:
            print(f"\nStopped - Data saved to {filename}")
            break
        except UnicodeDecodeError as e:
            print(f"Decoding error: {e}")
            continue

ser.close()