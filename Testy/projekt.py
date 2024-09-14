from microbit import i2c, pin8, pin12, pin14, pin15, button_a, button_b, sleep
from utime import ticks_ms, ticks_us, ticks_diff
from machine import time_pulse_us

MOTOR_I2C_ADDR = 0x70
TICKS_PER_CIRCLE = 40

class DirectionEnum:
    FORWARD = 1
    BACK =  2
    LEFT = 11
    RIGHT = 12
    UP = 21
    DOWN = 22

class UnitEnum:
    TicksPerSecond = 1
    CirclePerSecond = 2
    RadianPerSecond = 3
    MeterPerSecond = 4

class Velocity:
    def __init__(self):
        self.forward = 0.0
        self.angular = 0.0

class CalibrateFactors:
    def __init__(self, min_rychlost, min_pwm_rozjezd, min_pwm_dojezd, a, b):
        self.minimum_speed = min_rychlost
        self.minimum_pwm_when_stopped = min_pwm_rozjezd
        self.minimum_pwm_in_motion = min_pwm_dojezd
        self.a = a
        self.b = b

class SpeedTicks:
    # pocet desetin pro ktere si pamatujeme hodnoty
    LIMIT = 50
    def __init__(self):
        self.__index = -1        # prvni hodnota bude ulozena do indexu 0 (coz je -1 + 1)
        self.__times = [0] * self.LIMIT
        self.__ticks = [0] * self.LIMIT
        self.__countValues = -1  # ani prvni hodnotu nechci brat jako správnou (bude ignorovana)
        self.__lastTime = -1
        self.isStopped = True

    # Zjisti z casu jestli uz muzeme ulozit data do dalsiho indexu
    def getNewIndex(self, time: int):
        newTime = int(time / 100_000)                       # cas v microsekundach prevedu na desetiny sekundy
        if newTime == self.__lastTime:
            return -1
        else:
            self.__lastTime = newTime
            return (self.__index + 1) % self.LIMIT

    # Ulozime nove hodnoty do pole
    def nextValues(self, newIndex, time, ticks):
        if self.__countValues < self.LIMIT:                 # jeste nemame vsechny hodnoty pole zaplnene?
            self.__countValues += 1                         # ano -> pricteme ze mame dalsi hodnotu
            if self.__countValues > 10:
                self.isStopped = (self.__ticks[self.__index]-ticks) == 0
        self.__times[newIndex] = time
        self.__ticks[newIndex] = ticks
        self.__index = newIndex

    # Nech nas pracovat. Musime volat dostatecne casto
    def update(self, ticks):
        time = ticks_us()
        newIndex = self.getNewIndex(time)
        if newIndex >= 0:
            self.nextValues(newIndex, time, ticks)

    # spoci rychlost v  tikach za sekundu (count je pocet hodnot ktere pouzije pro vypocet, offset je pocet hodnot o ktere se posuneme do minulosti)
    def calculate(self, count=5, offset=0):
        if count < 2:
            count = 10
        if count+offset >= self.__countValues:
            count = self.__countValues - offset - 1
        if count < 2:
            # nemame dost hodnot na spocteni rychlosti -> predpokladame ze na zacatku stojime a vracimne proto rychlost 0
            return 0
        # zmenseni chyby, pokud nastane na krajnich hodnotach => zprumerujeme rychlost spoctenou z o 1 desetinu starsimi daty
        speed0 = self.__calculate(count, offset)
        speed1 = self.__calculate(count, offset+1)
        return  (speed0 + speed1) / 2

    # spocteni rychlosti za definovanych podminek (nejsou potreba kontroly, ty se provedli ve vnejsi funkci)
    def __calculate(self, count, offset):
        endIndex = (self.__index - offset) % self.LIMIT
        startIndex = (endIndex - count + 1) % self.LIMIT
        diffTimes = self.__times[endIndex] - self.__times[startIndex]
        diffTicks = self.__ticks[endIndex] - self.__ticks[startIndex]
        # rychlost = pocet tiku za 1s (tj. za 1_000_000 us)
        return 1_000_000 * diffTicks / diffTimes


class Encoder:
    def __init__(self, place):
        if place == DirectionEnum.LEFT:
            self.__pin = pin14
        else:
            self.__pin = pin15
        self.__speedTicks = SpeedTicks()
        self.__oldValue = self.readPin()
        self.ticks = 0
        self.direction = DirectionEnum.UP

    def isStopped(self):
        return self.__speedTicks.isStopped

    def readPin(self):
        return self.__pin.read_digital()

    def nextTick(self):
        if self.direction == DirectionEnum.UP:
            self.ticks += 1
            return 0
        if self.direction == DirectionEnum.DOWN:
            self.ticks -= 1
            return 0
        return -1

    # Nech nas pracovat. Musime volat dostatecne casto (idealne casteji nez budou prichazet tiky)
    def update(self, direction):
        self.direction = direction
        newValue = self.readPin()
        if (newValue != self.__oldValue):
            self.nextTick()
            self.__oldValue = newValue
        self.__speedTicks.update(self.ticks)

    def getSpeed(self, unit, count=5, offset=0):
        # nejpre spocti rychlost v tikach za sekundu
        speed = self.__speedTicks.calculate(count, offset)
        if unit == UnitEnum.TicksPerSecond:
            return speed
        # uprav rychlost na otacky za sekundu
        speed /= TICKS_PER_CIRCLE
        if unit == UnitEnum.CirclePerSecond:
            return speed
        # uprav rychlost na radiany za sekundu
        speed *= (2 * 3.1416)
        if unit == UnitEnum.RadianPerSecond:
            return speed
        # chceme nejakou jinou jednotku a to neumime spocitat
        return 0

class RegulatorP:
    def __init__(self, p, timeout_ms):
        self.__k = p
        self.__timeout_ms = timeout_ms
        self.__lastRegulationTime = ticks_ms()

    def isTimeout(self, time):
        diff = ticks_diff(time, self.__lastRegulationTime)
        return  diff >= self.__timeout_ms

    def getOutput(self, time, inputNominal, inputActual):
        self.__lastRegulationTime = time
        error = inputNominal - inputActual
        changeValue = self.__k * error
#        print("regulator.getOutput - input:", inputNominal, inputActual, error, changeValue)
        return changeValue

class Senzors:
    ObstaleRight = 0x40
    ObstaleLeft = 0x20
    LineTrackRight = 0x10
    LineTrackMiddle = 0x08
    LineTrackLeft = 0x04

    def __init__(self):
        self.__timeout_ms = 50
        self.readData()

    def readData(self, time=0):
        self.__data = i2c.read(0x38, 1)[0]
        if time == 0:
            self.__lastTimeRead = ticks_ms()
        else:
            self.__lastTimeRead = time

    def getSenzor(self, senzor):
        return (self.__data & senzor) == 0

    def isTimeout(self, time):
        diff = ticks_diff(time, self.__lastTimeRead)
        return  diff >= self.__timeout_ms

    def update(self):
        time = ticks_ms()
        if self.isTimeout(time):
            self.readData(time)


class Wheel:
    def __init__(self, place, radius, calibrateFactors):
        self.__place = place
        self.__encoder = Encoder(place)
        self.__regulator = RegulatorP(6, 500)
        self.__calibrateFactors = calibrateFactors
        self.radius = radius
        self.speed = 0.0
        self.direction = DirectionEnum.FORWARD
        if place == DirectionEnum.RIGHT:
            self.__pwmNoBack = 2
            self.__pwmNoForw = 3
        elif place == DirectionEnum.LEFT:
            self.__pwmNoBack = 4
            self.__pwmNoForw = 5
        else:
            self.__pwmNoBack = 0
            self.__pwmNoForw = 0
        i2c.write(MOTOR_I2C_ADDR, bytes([0x00, 0x01]))
        i2c.write(MOTOR_I2C_ADDR, bytes([0xE8, 0xAA]))

    def emergencyShutdown(self):
        self.speed = 0.0
        self.writePWM(self.__pwmNoBack, self.__pwmNoForw, 0)

    def isStopped(self):
        return self.__encoder.isStopped()

    def getMinimumSpeed(self):
        return self.__calibrateFactors.minimum_speed

    def writePWM(self, offPwmNo, onPwmNo, pwm):
        i2c.write(MOTOR_I2C_ADDR, bytes([offPwmNo, 0]))
        i2c.write(MOTOR_I2C_ADDR, bytes([onPwmNo, pwm]))
        self.__pwm = pwm
#        print("Wheel.writePWM:", pwm if self.direction == DirectionEnum.FORWARD else -pwm)
        return 0

    def getPwmFromSpeed(self, speed):
        if speed==0.0:
            return 0
        return self.__calibrateFactors.a * speed + self.__calibrateFactors.b

    def rideSpeed(self, speed):
        self.speed = speed
        if self.speed >= 0:
            self.direction = DirectionEnum.FORWARD
        else:
            self.direction = DirectionEnum.BACK
        pwm = self.getPwmFromSpeed(abs(speed))
#        print("wheel.rideSpeed:", self.speed, pwm, self.direction)
        self.__ridePwm(pwm)

    # pokud je rychlost kterou chceme jed ruzna od 0, musime korigovat pwm aby nekleslo pod minimalni hodnotu
    def checkMinimumPwm(self, pwm):
        if self.speed != 0.0:
            if (self.isStopped()):
                minimum_pwm = self.__calibrateFactors.minimum_pwm_when_stopped
            else:
                minimum_pwm = self.__calibrateFactors.minimum_pwm_in_motion
            if pwm < minimum_pwm:
                pwm = minimum_pwm
        return pwm

    def __ridePwm(self, pwm):
        origPwm = pwm
        pwm = int(pwm)
        pwm = self.checkMinimumPwm(pwm)
#        if self.__place == DirectionEnum.RIGHT:
#            print("Wheel.ridePwm - speed:", self.speed, "  pwm:", pwm, origPwm)
        if pwm < 0:                                                                     # zkontroluj nejmensi hodnotu
            return -1
        if pwm > 255:                                                                   # zkontroluj nejvetsi hodnotu
            return -2
        if self.__pwmNoForw>0 and self.__pwmNoBack>0:
            if self.direction == DirectionEnum.FORWARD:                                 # pokud jedeme dopredu
                return self.writePWM(self.__pwmNoBack, self.__pwmNoForw, pwm)           # -> nastav pwm dopredu
            if self.direction == DirectionEnum.BACK:                                    # pokud jedeme vzad
                return self.writePWM(self.__pwmNoForw, self.__pwmNoBack, pwm)           # -> nastav pwm dozadu
            return -3                                                                   # neznamy smer
        return -4                                                                       # spatne definovane kolo (ani leve ani prave)

    def __changePwm(self, changeValue):
        newPwm = 0
        if self.direction == DirectionEnum.FORWARD:
            newPwm = self.__pwm + changeValue
        if self.direction == DirectionEnum.BACK:
            newPwm = self.__pwm - changeValue
        if newPwm > 255:
            newPwm = 255
        if newPwm < 0:
            newPwm  = 0
        return self.__ridePwm(newPwm)

    def getSpeed(self, unit, count=5, offset=0):
        if unit == UnitEnum.MeterPerSecond:
            return self.radius * self.__encoder.getSpeed(UnitEnum.RadianPerSecond, count, offset)
        return self.__encoder.getSpeed(unit, count, offset)

    def regulate(self):
        time = ticks_ms()
        if self.__regulator.isTimeout(time):
            measureSpeed = self.__encoder.getSpeed(UnitEnum.RadianPerSecond)
            changeValue = self.__regulator.getOutput(time, self.speed, measureSpeed)
#            print("Wheel.regulate - speed:", measureSpeed, self.speed, "  changeValue:", changeValue)
            self.__changePwm(changeValue)

    def convertWhellDirectionToEncoder(self):
        return DirectionEnum.UP if self.direction == DirectionEnum.FORWARD else DirectionEnum.DOWN

    def update(self):
        self.__encoder.update(self.convertWhellDirectionToEncoder())
        self.regulate()

class MotionControl:
    def __init__(self, wheelbase, wheelDiameter, calibrateLeft, calibrateRight):
        self.__d = wheelbase / 2
        self.velocity = Velocity()
        self.__wheelLeft = Wheel(DirectionEnum.LEFT, wheelDiameter / 2, calibrateLeft)
        self.__wheelRight = Wheel(DirectionEnum.RIGHT, wheelDiameter / 2,  calibrateRight)

    def emergencyShutdown(self):
        try:
            self.newVelocity(0, 0)
        except BaseException as e:
            self.__wheelLeft.emergencyShutdown()
            self.__wheelRight.emergencyShutdown()
            raise e

    def getMinimumSpeed(self):
        minimumLeft = self.__wheelLeft.getMinimumSpeed()
        minimumRight = self.__wheelRight.getMinimumSpeed()
        return max(minimumLeft, minimumRight)

    def newVelocity(self, forward, angular):
        self.velocity.forward = forward
        self.velocity.angular = angular
        self.__wheelLeft.rideSpeed(self.velocity.forward - self.__d * self.velocity.angular)
        self.__wheelRight.rideSpeed(self.velocity.forward + self.__d * self.velocity.angular)

    def update(self):
        self.__wheelLeft.update()
        self.__wheelRight.update()

class Sonar:
    MAX_DISTANCE = 10

    def __init__(self, timeoutMeasure):
        self.__trigger = pin8
        self.__trigger.write_digital(0)
        self.__echo = pin12
        self.__echo.read_digital()
        self.__lastMeasureTime = 0
        self.__lastReturned = -3
        self.__timeout_ms = timeoutMeasure
        self.lastDistance = -1

    def calculateDistance(self, time):
        self.__lastMeasureTime = time
        speed = 340    # m/s
        self.__trigger.write_digital(1)
        self.__trigger.write_digital(0)
        time_us = time_pulse_us(self.__echo, 1, 5_000)
        if time_us < 0:
            return time_us
        time_s = time_us / 1_000_000
        distance = time_s * speed / 2
        return distance

    def isTimeout(self, time):
        diff = ticks_diff(time, self.__lastMeasureTime)
        return  diff >= self.__timeout_ms

    def update(self):
        time = ticks_ms()
        if self.isTimeout(time):
            self.__lastReturned = self.calculateDistance(time)
            if self.__lastReturned > 0:
                self.lastDistance = self.__lastReturned
            if self.__lastReturned == -1:
                self.lastDistance = self.MAX_DISTANCE

class Robot:
    def __init__(self, leftCalibrate, rightCalibrate):
        i2c.init(freq=400_000)
        self.__senzors = Senzors()
        self.__sonar = Sonar(300)
        self.__regulator = RegulatorP(50, 1_000)
        self.motionControl = MotionControl(0.15, 0.067, leftCalibrate, rightCalibrate)
        self.motionControl.newVelocity(1, 0)
        self.counterUpdate = 0

    def emergencyShutdown(self):
        self.motionControl.emergencyShutdown()

    def supplyVoltage(self):
        return 0.00898 * pin2.read_analog()

    def getObstacleDistance(self):
        return self.__sonar.lastDistance

    def testBumber(self):
        if self.__senzors.getSenzor(Senzors.ObstaleLeft) or self.__senzors.getSenzor(Senzors.ObstaleRight):
            self.motionControl.newVelocity(0, 0)

    def speedLimitation(self, speed):
        if speed >= 0:
            sign = 1
        else:
            sign = -1
        absSpeed = abs(speed)
        maxSpeed = 10
        minSpeed = self.motionControl.getMinimumSpeed()
        if absSpeed > maxSpeed:
            absSpeed = maxSpeed
        elif absSpeed < minSpeed:
            absSpeed = minSpeed
        return sign * absSpeed

    def regulateSpeed(self):
        time = ticks_ms()
        if self.__regulator.isTimeout(time):
            self.counterUpdate += 1
            distance = self.getObstacleDistance()
            distanceDif = abs(distance-0.2)
            if distanceDif < 0.01:      # prilis mala odchylka?
                newSpeedForward = 0     # ano -> uz to nechame byt
                newSpeedLimit = 0
            else:                       # regulujeme rychlost podle rozdilu vzdalenosti od prekazky
                newSpeedForward = self.__regulator.getOutput(time, -0.2, -distance)
                newSpeedLimit = self.speedLimitation(newSpeedForward)
            self.motionControl.newVelocity(newSpeedLimit, 0)
#            print("Distance:", distance, distanceDif, "  SpeedControl:", newSpeedForward, "/", newSpeedLimit, "SpeedVheel:", self.motionControl.__wheelLeft.speed, "/", self.motionControl.__wheelLeft.getSpeed(UnitEnum.RadianPerSecond), "|", self.motionControl.__wheelRight.speed, "/", self.motionControl.__wheelRight.getSpeed(UnitEnum.RadianPerSecond))

    def update(self):
        self.motionControl.update()
        self.__senzors.update()
        self.__sonar.update()
        self.testBumber()
        self.regulateSpeed()

def main():
    leftCalibrate  = CalibrateFactors(3.0, 110, 75, 11.692, 28.643)
    rightCalibrate = CalibrateFactors(3.0, 110, 75, 12.259, 30.332)
    robot = Robot(leftCalibrate, rightCalibrate)
    try:
        speed = 4
        robot.motionControl.newVelocity(speed, 0)
        lastPrint = 0
        while not button_a.was_pressed():
            if button_b.was_pressed():
                speed *= -1
                robot.motionControl.newVelocity(speed, 0)
            robot.update()
            time = ticks_ms()
            diff = ticks_diff(time, lastPrint)
            if diff > 1_000:
                lastPrint = time
                speedL = robot.motionControl.__wheelLeft.getSpeed(UnitEnum.RadianPerSecond)
                speedR = robot.motionControl.__wheelRight.getSpeed(UnitEnum.RadianPerSecond)
#                print(speedL, speedR, robot.__sonar.lastDistance, robot.counterUpdate)
            sleep(1)
        robot.motionControl.newVelocity(0, 0)
    except BaseException as e:
        print("---------- Nastala nejaka chyba -> vsechno vypnout!")
        robot.emergencyShutdown()
        raise e

if __name__ == "__main__":
    main()
