import subprocess
import threading
import re

imu_process = subprocess.Popen(
    ["lidar/build/lidar", "--port", "/dev/ttyUSB0", "--baud", "230400", "--print", "--window", "200x200", "--min-distance", "0.1", "--max-distance", "2", "--update-delay", "0.2", "--dots", "360"],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True
)

lidar_distances_tmp = [0.0]*360

lidar_distances = [0.0]*360

def imu_thread():
    global lidar_distances_tmp, lidar_distances
    for line in imu_process.stdout:
        # print("LIDAR > ", line.strip())
        if(line.startswith("bin[")):
            match = re.search(r"bin\[(\d+)\].*?range=([\d.]+)", line)
            deg = int(match.group(1))
            range = float(match.group(2))
            lidar_distances_tmp[deg] = range
        elif(line.startswith("=== end scan ===")):
            lidar_distances = lidar_distances_tmp.copy()
            # print(lidar_distances)
       

threading.Thread(target=imu_thread).start()

def getDistances():
    print(lidar_distances)
    slice_size = 20
    front_slice = lidar_distances[0:slice_size] + lidar_distances[359-slice_size:359]
    right_slice = lidar_distances[90-slice_size:90+slice_size]
    left_slice = lidar_distances[270-slice_size:270+slice_size]
    front_slice = [x for x in front_slice if x > 0]
    right_slice = [x for x in right_slice if x > 0]
    left_slice = [x for x in left_slice if x > 0]
    front = 2
    right = 0
    left = 0
    if(len(front_slice) != 0):
        front = sum(front_slice)/len(front_slice)
    if(len(right_slice) != 0):
        right = sum(right_slice)/len(right_slice)
    if(len(left_slice) != 0):
        left = sum(left_slice)/len(left_slice)
    print(left, ", ",front,", ", right)
    return left,front,right