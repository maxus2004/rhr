# ничего на роботах нормально не работает
# по этому едем по правилу правой руки
# как школьники которые в первый раз Arduino увидели

import time
import motors
import imu
import lidar
import threading
from params import *

targetYaw = 0

def continueDriving():
    motors.motorCommand(LINEAR_SPEED,LINEAR_SPEED)

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
        if(time.time()-prevMoveTime > UNSTUCK_INTERVAL):
            prevMoveTime = time.time()
            if prevMoveDirection == "forwards":
                motors.motorCommand(-UNSTUCK_SPEED, -UNSTUCK_SPEED)
                prevMoveDirection = "backwards"
            else:
                motors.motorCommand(UNSTUCK_SPEED, UNSTUCK_SPEED)
                prevMoveDirection = "forwards"
            time.sleep(0.1)

        # turn
        if(error > 1):
            motors.motorCommand(TURN_SPEED, -TURN_SPEED)
        elif(error < -1):
            motors.motorCommand(-TURN_SPEED, TURN_SPEED)
        else:
            break
        time.sleep(TURN_INTERVAL)

def turnRight():
    global targetYaw
    targetYaw = fixAngleOverflow(targetYaw+90)
    turnTo(targetYaw)

def turnLeft():
    global targetYaw
    targetYaw = fixAngleOverflow(targetYaw-90)
    turnTo(targetYaw)

print("calibrationg IMU...")
time.sleep(10)
print("done calibrating")

while True:
    leftDistance, frontDistance, rightDistance = lidar.getDistances()

    # duh
    if leftDistance < RIGHT_DISTANCE:
        # drive forward a bit
        startTime = time.time()
        while(time.time()-startTime < RIGHT_TURN_DELAY):
            continueDriving()
            time.sleep(DRIVE_INTERVAL)
        # if still can turn left, turn left
        print("TURNING LEFT")
        leftDistance, frontDistance, rightDistance = lidar.getDistances()
        if leftDistance > RIGHT_DISTANCE: turnLeft()
        stop()
        time.sleep(TURN_DELAY)
    elif frontDistance < FRONT_DISTANCE:
        print("TURNING RIGHT")
        turnRight()
        stop()
        time.sleep(TURN_DELAY)

    continueDriving()
    time.sleep(DRIVE_INTERVAL)