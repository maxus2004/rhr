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
                print(f"< {line}")
                log_line("IN", line)
        except Exception as e:
            print(f"Read error: {e}")
            log_line("ERROR", str(e))

# Start reader thread
threading.Thread(target=read_loop, daemon=True).start()

# Main input loop
while True:
    try:
        cmd = input("> ").strip()
        msg = ""
        match cmd:
            case "w":
                msg = '{"T":1,"L":100,"R":100}'
            case "s":
                msg = '{"T":1,"L":-100,"R":-100}'
            case "a":
                msg = '{"T":1,"L":-500,"R":500}'
            case "d":
                msg = '{"T":1,"L":500,"R":-500}'
            case "":
                msg = '{"T":1,"L":0,"R":0}'
            case "start":
                msg = '{"T":131,"cmd":1}'
            case "stop":      
                msg = '{"T":131,"cmd":0}'
            case "exit":
                print("Exiting...")
                log_file.close()
                ser.close()
                break

        if msg:
            ser.write((msg + "\r\n").encode())
            log_line("OUT", msg)
            print(f"> {msg}")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
        log_file.close()
        ser.close()
        break