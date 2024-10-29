from project import Robot, Constants, CalibrateFactors
from HardwarePlatform import sleep, ticks_ms, ticks_us, ticks_diff
from picoed import display
from picoed import button_a, button_b


if __name__ == "__main__":

    leftCalibrate = CalibrateFactors(0.5, 130, 80, 11.692, 28.643)
    rightCalibrate = CalibrateFactors(0.5, 130, 80, 12.259, 26.332)
    robot = None
    try:
        robot = Robot(leftCalibrate, rightCalibrate)
        while True:
            while not button_b.was_pressed():
                if button_a.was_pressed():
                    robot.motionControl.calibration(90, 220, 1)
                    display.scroll("   ")
                robot.update()
                sleep(1)
                
            while not button_a.was_pressed():
                robot.update()
#                robot.rideLine()
                robot.rideLine(regulatedDistance=0.2)
                sleep(1)
                if robot.getSituationLine() != Constants.Line:
                    # pokud nejsme na care koncime jizdu
                    break
            robot.motionControl.newVelocity(0, 0)
            print("Stop")
    except BaseException as e:
        if robot:
            robot.emergencyShutdown()
        print("Exception")
        raise e
