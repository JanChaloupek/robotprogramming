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
#        robot.motionControl.newVelocity(0, 0)
#        robot.motionControl.odometry.print()
        display.scroll(str(pointsIndex)+"-"+str(pointsIndex+1))
        state = Constants.ST_Turn
#        print("-------- st_turn")

    if state == Constants.ST_Turn:
        isDone, x = robot.follow(getActualPoint(), False)
        if isDone:
#            robot.motionControl.newVelocity(0, 0)
            state = Constants.ST_Follow
#            print("-------- st_follow")

    if state == Constants.ST_Follow:
        isTurn, isDone = robot.follow(getActualPoint(), True)
        if isDone:
#            robot.motionControl.newVelocity(0, 0)
            state = Constants.ST_NextPoint
#            print("-------- st_nextPoint")
        elif isTurn:
#            robot.motionControl.newVelocity(0, 0)
            state = Constants.ST_Turn
            print("-------- st_turn (spatny smer)")

    if state == Constants.ST_NextPoint:
        nextPoint()
        if not getActualPoint():
            state = Constants.ST_Success
#            print("-------- st_success")
        else:
            state = Constants.ST_Start
#            print("-------- st_start ")

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
    print("Start")
    points = [
        [ 0.77, 0.00],
        [ 0.92,-0.20],
        [ 0.75,-0.40],
        [ 0.20,-0.40],
        [-0.20,-0.00],
    ]
    pointsIndex = 0

    leftCalibrate = CalibrateFactors(0.5, 130, 80, 11.692, 28.643)
    rightCalibrate = CalibrateFactors(0.5, 130, 80, 12.259, 26.332)
    robot = None
    try:
        robot = Robot(leftCalibrate, rightCalibrate)
        state = Constants.ST_Start
        stateExecComm = Constants.ST_Start
        display.scroll("0")
        while not button_b.was_pressed():
            robot.update()
            
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
        robot.motionControl.odometry.print()
    except BaseException as e:
        if robot:
            robot.emergencyShutdown()
        print("Exception")
        raise e
