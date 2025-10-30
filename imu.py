import subprocess
import threading
import os

imu_process = subprocess.Popen(
    ["imu/build/imu"],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    preexec_fn=lambda: os.nice(-10)
)

imu_yaw = 0

def imu_thread():
    global imu_yaw
    for line in imu_process.stdout:
        # print("IMU > ", line.strip())
        if line.startswith("yaw:"):
            imu_yaw = float(line.strip().split()[1])

threading.Thread(target=imu_thread).start()

def getYaw():
    return imu_yaw