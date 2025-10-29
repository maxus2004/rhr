import subprocess
import threading
import re

imu_process = subprocess.Popen(
    ["lidar/build/lidar", "--port", "/dev/ttyUSB0", "--baud", "230400", "--print", "--window", "200x200", "--min-distance", "0.2", "--max-distance", "2", "--update-delay", "0.2", "--dots", "360"],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

lidar_distances_tmp = [0.0]*360

lidar_distances = [0.0]*360

def imu_thread():
    global lidar_distances_tmp, lidar_distances
    for line in imu_process.stdout:
        print("LIDAR > ", line.strip())
        if(line.startswith("bin[")):
            match = re.search(r"bin\[(\d+)\].*?range=([\d.]+)", line)
            deg = int(match.group(1))
            range = float(match.group(2))
            lidar_distances_tmp[deg] = range
        elif(line.startswith("=== end scan ===")):
            lidar_distances = lidar_distances_tmp.copy()
            print(lidar_distances)
       

threading.Thread(target=imu_thread).start()

def getDistances():
    return 0,0,0