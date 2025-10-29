import subprocess

imu_process = subprocess.Popen(
    ["imu/build/imu"],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

imu_yaw = 0

for line in imu_process.stdout:
    print("IMU > ", line.strip())
    if line.startswith("yaw:"):
        imu_yaw = float(line.strip().split()[1])

def getYaw():
    return imu_yaw