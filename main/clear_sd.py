#!/usr/bin/env python3
import serial
import time
import sys

def clear_sd_card(port='/dev/ttyACM0', baudrate=115200):
    try:
        # Open serial connection
        ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        
        # Send clear command
        ser.write(b'c\n')
        
        # Wait for response
        time.sleep(1)
        
        # Read all available data
        while True:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8').strip()
                print(line)
                if "SD card cleared successfully" in line:
                    break
            time.sleep(0.1)
            
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    # Get port from command line argument if provided
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    clear_sd_card(port) 