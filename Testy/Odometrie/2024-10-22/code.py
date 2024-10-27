from project import Robot, Constants, CalibrateFactors
from HardwarePlatform import sleep, ticks_ms, ticks_us, ticks_diff
from picoed import display
from picoed import button_a
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

    if state == Constants.ST_Start:
        state = Constants.ST_CalculateAngular

    if state == Constants.ST_CalculateAngular:
        state = Constants.ST_Turn
        
    if state == Constants.ST_Turn:
        state = Constants.ST_Follow        

    if state == Constants.ST_Follow:
        isDone = robot.follow(getActualPoint())
        if isDone:
            state = Constants.ST_NextPoint

    if state == Constants.ST_NextPoint:
        nextPoint()
        if not getActualPoint():
            state = Constants.ST_Success

    if state == Constants.ST_Success:
        robot.motionControl.newVelocity(0, 0)
        return True

    if state == Constants.ST_Failure:
        robot.motionControl.newVelocity(0, 0)
        return True

    return False

if __name__ == "__main__":

    print("Start")
    points = [
        [0.05, 20],
    ]
    pointsIndex = 0
    
    leftCalibrate = CalibrateFactors(1.0, 110, 70, 11.692, 28.643)
    rightCalibrate = CalibrateFactors(1.0, 110, 70, 12.259, 30.332)
    robot = None
    try:
        robot = Robot(leftCalibrate, rightCalibrate)
        state = Constants.ST_Start
        stateExecComm = Constants.ST_Start
        while not button_a.was_pressed():
            robot.update()
            isEnd = stateMachine()
            if isEnd:
                break
            sleep(1)
        robot.motionControl.newVelocity(0, 0)
        # pockej chvilku na pripadne posledni spocteni a zobrazeni polohy
        sleep(1_000)
        robot.motionControl.update()
        print("Stop")
        print(robot.motionControl.odometry.x, robot.motionControl.odometry.y, robot.motionControl.odometry.theta)
    except BaseException as e:
        if robot:
            robot.emergencyShutdown()
        print("Exception")
        raise e
