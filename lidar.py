import subprocess
import threading

imu_process = subprocess.Popen(
    ["lidar/build/lidar", "--port", "/dev/ttyUSB0", "--baud", "230400", "--print"],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

def imu_thread():
    global imu_yaw
    for line in imu_process.stdout:
        print("LIDAR > ", line.strip())
       

threading.Thread(target=imu_thread).start()

def getDistances():
    return 0,0,0