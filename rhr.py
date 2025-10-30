# ничего на роботах нормально не работает
# по этому едем по правилу правой руки
# как школьники которые в первый раз Arduino увидели

import time
import motors
import imu
import lidar
import threading

targetYaw = 0

def continueDriving():
    motors.motorCommand(300,300)

def stop():
    motors.motorCommand(0,0)

def fixAngleOverflow(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a

def turnTo(targetYaw):
    prevMoveTime = time.time()
    prevMoveDirection = "forwards"
    while True:
        yaw = imu.getYaw()
        error = fixAngleOverflow(targetYaw-yaw)

        # move forward/backward a bit if stuck
        if(time.time()-prevMoveTime > 2):
            prevMoveTime = time.time()
            if prevMoveDirection == "forwards":
                motors.motorCommand(-300, -300)
                prevMoveDirection = "backwards"
            else:
                motors.motorCommand(300, 300)
                prevMoveDirection = "forwards"
            time.sleep(0.1)

        # turn
        if(error > 1):
            motors.motorCommand(500, -500)
        elif(error < -1):
            motors.motorCommand(-500, 500)
        else:
            break
        time.sleep(0.02)

def turnRight():
    global targetYaw
    targetYaw = fixAngleOverflow(targetYaw+90)
    turnTo(targetYaw)

def turnLeft():
    global targetYaw
    targetYaw = fixAngleOverflow(targetYaw-90)
    turnTo(targetYaw)


time.sleep(10)

while True:
    leftDistance, frontDistance, rightDistance = lidar.getDistances()

    # duh
    if rightDistance > 0.4:
        # drive forward a bit
        startTime = time.time()
        while(time.time()-startTime < 2):
            continueDriving()
            time.sleep(0.1)
        # if still can turn right, turn right
        print("TURNING RIGHT")
        leftDistance, frontDistance, rightDistance = lidar.getDistances()
        if rightDistance > 0.4: turnRight()
        stop()
        time.sleep(1)
    elif frontDistance < 0.2:
        print("TURNING LEFT")
        turnLeft()
        stop()
        time.sleep(1)

    continueDriving()
    time.sleep(0.1)