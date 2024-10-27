from project import Robot, Constants, CalibrateFactors
from HardwarePlatform import sleep, ticks_ms, ticks_us, ticks_diff
from picoed import display
from picoed import button_a, button_b
from math import sin, cos

def getActualPoint():
    global points
    global pointsIndex
    if pointsIndex >= len(points):
        return None
    return points[pointsIndex]

def nextPoint():
    global pointsIndex
    pointsIndex += 1

def stateMachine():
    global state
    global robot
    global theta
    global startTurn
    global pointsIndex

    if state == Constants.ST_Start:
        display.scroll(str(pointsIndex)+"-"+str(pointsIndex+1))
        state = Constants.ST_Turn

    if state == Constants.ST_Turn:
        isDone, x = robot.follow(getActualPoint(), False)
        if isDone:
            state = Constants.ST_Follow

    if state == Constants.ST_Follow:
        isTurn, isDone = robot.follow(getActualPoint(), True)
        if isDone:
            state = Constants.ST_NextPoint
        elif isTurn:
            state = Constants.ST_Turn

    if state == Constants.ST_NextPoint:
        nextPoint()
        if not getActualPoint():
            state = Constants.ST_Success
        else:
            state = Constants.ST_Start

    if state == Constants.ST_Success:
        robot.motionControl.newVelocity(0, 0)
        display.scroll("End")
        return True

    if state == Constants.ST_Failure:
        robot.motionControl.newVelocity(0, 0)
        display.scroll("Err")
        return True

    return False

if __name__ == "__main__":
    display.scroll("Run")
    points = [
        [ 0.78, 0.02],
        [ 1.00,-0.21],
        [ 0.78,-0.42],
        [ 0.25,-0.42],
        [-0.20,-0.05],
    ]
    pointsIndex = 0

    leftCalibrate = CalibrateFactors(0.5, 130, 80, 11.692, 28.643)
    rightCalibrate = CalibrateFactors(0.5, 130, 80, 12.259, 26.332)
    robot = None
    try:
        robot = Robot(leftCalibrate, rightCalibrate)
        while True:
            display.scroll("0")      
            while not button_b.was_pressed():
                if button_a.was_pressed():
                    robot.motionControl.calibration(90, 220, 1)
                    display.scroll("0")      
                robot.update()
                sleep(1)
    
            robot.motionControl.reinitOdometry()
            state = Constants.ST_Start
            stateExecComm = Constants.ST_Start
            pointsIndex = 0            
            
            while not button_a.was_pressed():
                robot.update()
                isEnd = stateMachine()
                if isEnd:
                    break
                sleep(1)
            robot.motionControl.newVelocity(0, 0)
            # pockej chvilku na pripadne posledni spocteni a zobrazeni polohy
            for x in range(200):
                robot.update()
                sleep(1)
            print("Stop")
            robot.motionControl.odometry.print()    
    except BaseException as e:
        if robot:
            robot.emergencyShutdown()
        print("Exception")
        raise e
