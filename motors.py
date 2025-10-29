import serial
import threading
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
                print(f"ESP32 > {line}")
                log_line("IN", line)
        except Exception as e:
            print(f"Read error: {e}")
            log_line("ERROR", str(e))

# Start reader thread
threading.Thread(target=read_loop, daemon=True).start()

def command(msg):
    ser.write((msg + "\r\n").encode())
    print(f"ESP32 < {msg}")
    log_line("OUT", msg)

command('{"T":131,"cmd":1}')
command('{"T":3,"lineNum":0,"Text":" #          #  "}')
command('{"T":3,"lineNum":1,"Text":"# #  ##  ## ## "}')
command('{"T":3,"lineNum":2,"Text":"### #   #   # #"}')
command('{"T":3,"lineNum":3,"Text":"# # #    ## # #"}')

def motorCommand(left, right):
    command('{"T":1,"L":'+str(left)+',"R":'+str(right)+'}')