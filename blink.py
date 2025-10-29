import serial
import threading
import time
from datetime import datetime

# Open serial port
ser = serial.Serial(port='/dev/ttyACM1', baudrate=115200, timeout=1)

# Open log file in append mode
log_file = open("esp32_data.txt", "a", buffering=1)  # line-buffered

def log_line(prefix, line):
    """Write a timestamped line to log file."""
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    log_file.write(f"[{timestamp}] {prefix}: {line}\n")

def read_loop():
    """Continuously read from serial and log output."""
    while True:
        try:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print(f"< {line}")
                log_line("IN", line)
        except Exception as e:
            print(f"Read error: {e}")
            log_line("ERROR", str(e))

# Start reader thread
threading.Thread(target=read_loop, daemon=True).start()


ser.write(b'{"T":132,"IO1":255,"IO2":255}\r\n')
time.sleep(1)
ser.write(b'{"T":132,"IO1":0,"IO2":0}\r\n')
