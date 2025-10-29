# ничего на роботах нормально не работает
# по этому едем по правилу правой руки
# как школьники которые в первый раз Arduino увидели

import time
import motors
import imu

targetYaw = 0


def getDistances():
    return 0,0,0

def continueDriving():
    motors.motorCommand(100,100)

def fixAngleOverflow(a):
    while a > 180:
        a -= 360
    while a < -180:
        a += 360

def turnTo():
    prevMoveTime = time.time()
    prevMoveDirection = "forwards"
    while True:
        yaw = imu.getYaw()
        error = fixAngleOverflow(targetYaw-yaw)

        # move forward/backward a bit if stuck
        if(time.time()-prevMoveTime > 5):
            prevMoveTime = time.time()
            if prevMoveDirection == "forwards":
                motors.motorCommand(-100, -100)
                prevMoveDirection = "backwards"
            else:
                motors.motorCommand(100, 100)
                prevMoveDirection = "forwards"
            time.sleep(0.1)

        # turn
        if(error > 20):
            motors.motorCommand(500, -500)
        elif(error < -20):
            motors.motorCommand(-500, 500)
        if(error > 5):
            motors.motorCommand(100, -100)
        elif(error < -5):
            motors.motorCommand(-100, 100)
        else:
            break
        time.sleep(0.02)

def turnRight():
    # drive forward a bit
    startTime = time.time()
    while(time.time()-startTime < 1):
        continueDriving()
        time.sleep(0.02)

    # turn
    targetYaw = fixAngleOverflow(targetYaw+90)
    turnTo(targetYaw)

def turnLeft():
    # drive forward a bit
    startTime = time.time()
    while(time.time()-startTime < 1):
        continueDriving()
        time.sleep(0.02)

    # turn
    targetYaw = fixAngleOverflow(targetYaw-90)
    turnTo(targetYaw)


while True:
    leftDistance, frontDistance, rightDistance = getDistances()

    # duh
    if rightDistance > 0.2:
        turnRight()
    elif frontDistance < 0.2:
        turnLeft()

    continueDriving()
    time.sleep(0.02)