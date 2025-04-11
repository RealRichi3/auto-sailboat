import serial
import csv
import time
from datetime import datetime
import os
import glob

def find_arduino_port():
    """Find the Arduino port by checking common port names"""
    ports = glob.glob('/dev/tty[A-Za-z]*')
    for port in ports:
        if 'ACM' in port or 'USB' in port:
            try:
                s = serial.Serial(port)
                s.close()
                return port
            except (OSError, serial.SerialException):
                pass
    return None

def ensure_logs_directory():
    """Create logs directory if it doesn't exist"""
    if not os.path.exists('logs'):
        os.makedirs('logs')

def read_arduino_log(port=None, baudrate=115200):
    try:
        # Create logs directory
        ensure_logs_directory()
        
        # Generate timestamp for filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_file = os.path.join('logs', f'sailboat_log_{timestamp}.csv')
        
        # If no port specified, try to find Arduino
        if port is None:
            port = find_arduino_port()
            if port is None:
                print("No Arduino found. Please specify the port manually.")
                return
            print("Found Arduino on port: {}".format(port))
        
        # Try to open the port
        try:
            ser = serial.Serial(port, baudrate, timeout=10)
        except serial.SerialException:
            print("Port {} is busy. Please close any other programs using this port.".format(port))
            return
        
        print("Connected to {} at {} baud".format(port, baudrate))
        
        # Wait for Arduino to initialize
        time.sleep(2)
        
        # Send 'r' command to read log file
        ser.write(b'r')
        print("Sent read command")
        
        # Wait for start marker
        start_marker = "===LOG_FILE_START==="
        end_marker = "===LOG_FILE_END==="
        
        # Read until start marker
        while True:
            line = ser.readline().decode('utf-8').strip()
            if start_marker in line:
                break
            print("Waiting for start marker... Received: {}".format(line))
        
        # Read data until end marker
        log_data = []
        header = None
        
        while True:
            line = ser.readline().decode('utf-8').strip()
            if end_marker in line:
                break
            if line:  # Skip empty lines
                if header is None:
                    # Enhance header with units
                    header = line.split(',')
                    header = [col.strip() for col in header]
                    # Add units to headers
                    header = [f"{col} (degrees)" if "heading" in col.lower() else 
                             f"{col} (degrees)" if "direction" in col.lower() else
                             f"{col} (m/s)" if "speed" in col.lower() else
                             f"{col} (m)" if "distance" in col.lower() else
                             f"{col} (s)" if "time" in col.lower() else
                             col for col in header]
                else:
                    log_data.append(line.split(','))
        
        # Save to CSV file
        with open(output_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            # Write metadata
            writer.writerow(['Log File Information'])
            writer.writerow(['Timestamp', timestamp])
            writer.writerow(['Port', port])
            writer.writerow(['Baud Rate', baudrate])
            writer.writerow([])  # Empty row for separation
            # Write data
            writer.writerow(header)
            writer.writerows(log_data)
        
        print("\nLog data saved to {}".format(output_file))
        print("Total records: {}".format(len(log_data)))
        
        # Display sample data
        print("\nSample data:")
        print("------------")
        for i, row in enumerate(log_data[:5]):  # Show first 5 rows
            print("Record {}:".format(i+1))
            for h, v in zip(header, row):
                print("  {}: {}".format(h, v))
            print()
        
        # Close serial connection
        ser.close()
        
    except serial.SerialException as e:
        print("Error opening serial port: {}".format(e))
    except Exception as e:
        print("An error occurred: {}".format(e))

if __name__ == "__main__":
    # Try to automatically find the Arduino port
    read_arduino_log() 