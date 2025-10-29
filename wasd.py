# ничего на роботах нормально не работает
# по этому едем по правилу правой руки
# как школьники которые в первый раз Arduino увидели

import time
import motors
import imu
import threading

targetYaw = 0


def getDistances():
    return 0,0,0

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
            time.sleep(0.5)

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
    # drive forward a bit
    startTime = time.time()
    while(time.time()-startTime < 1):
        continueDriving()
        time.sleep(0.02)

    # turn
    targetYaw = fixAngleOverflow(targetYaw+90)
    turnTo(targetYaw)

def turnLeft():
    global targetYaw
    # drive forward a bit
    startTime = time.time()
    while(time.time()-startTime < 1):
        continueDriving()
        time.sleep(0.02)

    # turn
    targetYaw = fixAngleOverflow(targetYaw-90)
    turnTo(targetYaw)


state = "stop"

def drive_loop():
    global state
    while True:
        leftDistance, frontDistance, rightDistance = getDistances()

        if state == "go":
            continueDriving()
        if state == "back":
            motors.motorCommand(-300,-300)
        elif state == "right":
            turnRight()
            state = "go"
        elif state == "left":
            turnLeft()
            state = "go"
        else:
            stop()

        time.sleep(0.02)

threading.Thread(target=drive_loop).start()

while True:
    c = input()
    match c:
        case "w":
            state = "go"
        case "s":
            state = "back"
        case "a":
            state = "left"
        case "d":
            state = "right"
        case _:
            state = "stop"