from neopixel import NeoPixel
from microbit import (
    i2c,
    pin0,
#    pin8,
#    pin12,
    pin14,
    pin15,
    button_a,
    button_b,
    sleep,
    display,
)
from utime import ticks_ms, ticks_us, ticks_diff
from machine import time_pulse_us
import gc

MOTOR_I2C_ADDR = 0x70
TICKS_PER_CIRCLE = 40
TwoPI = 2 * 3.141592


class Constants:
    NONE = 0
    # Direction:
    LEFT = 1
    RIGHT = 2
    FORWARD = 11
    BACK = 12
    # HeadLightEnum:
    DippedBeams = 31
    HighBeams = 32
    # jednotky rychlosti
    TicksPerSecond = 41
    CirclePerSecond = 42
    RadianPerSecond = 43
    MeterPerSecond = 44
    # vysledky sledovani cary
    Line = 51
    CrossRoads = 52

    ST_Start = 100
    ST_Success = 101
    ST_Failure = 102

    ST_RidingLine = 111
    ST_DetectCrossRoads = 112
    ST_LocalizeBeforeExecCommand = 113
    ST_ExecuteCommand = 114
    ST_LocalizeAfterExecCommand = 116

    ST_EC_ExitCrossRoads = 121
    ST_EC_Turn = 122
    ST_EC_TurnToMiddleSenzor = 123


class Velocity:
    # třída pro uložení požadované rychlosti robota
    def __init__(self):
        self.forward = 0.0
        self.angular = 0.0


class CalibrateFactors:
    # třída pro uložení kalibračních hodnot pro motor
    def __init__(self, minSpeed, minPwmWhenStopped, minPwmInMotion, a, b):
        self.minSpeed = minSpeed
        self.minPwmWhenStopped = minPwmWhenStopped
        self.minPwmInMotion = minPwmInMotion
        self.a = a
        self.b = b


class RegulatorP:
    # třída implementující P-regulátor
    def __init__(self, p, timeout_ms):
        self.__k = p
        self.__timeout_ms = timeout_ms
        self.__lastRegulationTime = ticks_ms()

    def isTimeout(self, time):
        # vypršel čas pro další regulaci?
        diff = ticks_diff(time, self.__lastRegulationTime)
        return diff >= self.__timeout_ms

    def getActionIntervention(self, time, inputNominal, inputActual):
        # vypočti akční zásah
        self.__lastRegulationTime = time
        error = inputNominal - inputActual
        changeValue = self.__k * error
        return changeValue


class Senzors:
    # třída vyčítající senzory po i2c a jejich získání dotazem
    ObstaleRight = 0x40
    ObstaleLeft = 0x20
    LineTrackRight = 0x10
    LineTrackMiddle = 0x08
    LineTrackLeft = 0x04
    LineTrackAll = 0x1C

    def __init__(self):
        self.__timeout_ms = 33
        self.__timeNotLine = 0
        self.readData()

    def readData(self, time=0):
        # přečti data po i2c
        self.__data = i2c.read(0x38, 1)[0]
        if time == 0:
            self.__lastTimeRead = ticks_ms()
        else:
            self.__lastTimeRead = time

    def getData(self, mask):
        return self.__data & mask

    def getSenzor(self, senzor):
        # vrat stav jednoho senzoru z vyčtených dat
        return self.getData(senzor) == 0

    def isTimeout(self, time):
        # už je čas znovu vyčíst data senzoru?
        diff = ticks_diff(time, self.__lastTimeRead)
        return diff >= self.__timeout_ms

    def update(self):
        time = ticks_ms()
        if self.isTimeout(time):
            self.readData(time)

    def getSituationLine(self):
#        print(self.getData(Senzors.LineTrackAll))
        countLineTrack = 0
        if not self.getSenzor(Senzors.LineTrackLeft):
            countLineTrack += 1
        if not self.getSenzor(Senzors.LineTrackMiddle):
            countLineTrack += 1
        if not self.getSenzor(Senzors.LineTrackRight):
            countLineTrack += 1
#        print(countLineTrack)
        if countLineTrack >= 2:
            self.__timeNotLine = 0
            return Constants.CrossRoads
        if countLineTrack == 1:
            self.__timeNotLine = 0
            return Constants.Line
        if self.__timeNotLine == 0:
            self.__timeNotLine = ticks_ms()
        if ticks_diff(ticks_ms(), self.__timeNotLine) <= 1_000:
            return Constants.Line
        return Constants.NONE

class SpeedTicks:
    # Třída počítající rychlost z uložené historie tiků
    LIMIT = 20

    def __init__(self):
        self.__index = -1
        self.__times = [0] * self.LIMIT
        self.__ticks = [0] * self.LIMIT
        self.__countValues = -1
        self.__lastTime = -1
        self.isStopped = True

    def getNewIndex(self, time: int):
        # Zjisti z casu jestli uz muzeme ulozit další data do historie
        newTime = int(time / 100_000)
        if newTime == self.__lastTime:
            return -1
        else:
            self.__lastTime = newTime
            return (self.__index + 1) % self.LIMIT

    def isZeroChangeTicks(self, ticks):
        diff = self.__ticks[self.__index] - ticks
        return diff == 0

    def nextValues(self, newIndex, time, ticks):
        # Ulož další data do historie
        if self.__countValues < self.LIMIT:
            self.__countValues += 1
        if self.__countValues > 2:
            self.isStopped = self.isZeroChangeTicks(ticks)
        self.__times[newIndex] = time
        self.__ticks[newIndex] = ticks
        self.__index = newIndex

    def update(self, ticks):
        time = ticks_us()
        newIndex = self.getNewIndex(time)
        if newIndex >= 0:
            self.nextValues(newIndex, time, ticks)

    def calculate(self, count, offset):
        # Spočti rychlost v tikách za sekundu.
        # Použij na to count dat z historie a použij ty, které jsou offset staré
        if count < 2:
            count = 10
        if count + offset >= self.__countValues:
            count = self.__countValues - offset - 1
        if count < 2:
            return 0
        speed0 = self.__calculate(count, offset)
        speed1 = self.__calculate(count, offset + 1)
        return (speed0 + speed1) / 2

    def __calculate(self, count, offset):
        # Skutečné spočtení (bez kontrol a průměrování)
        endIndex = (self.__index - offset) % self.LIMIT
        startIndex = (endIndex - count + 1) % self.LIMIT
        diffTimes = self.__times[endIndex] - self.__times[startIndex]
        diffTicks = self.__ticks[endIndex] - self.__ticks[startIndex]
        return 1_000_000 * diffTicks / diffTimes


class Encoder:
    # Třída počítající tiky enkoderu
    def __init__(self, place, ticksCount, radius):
        if place == Constants.LEFT:
            self.__pin = pin14
        else:
            self.__pin = pin15
        self.__ticksCount = ticksCount
        self.__radius = radius
        self.__speedTicks = SpeedTicks()
        self.__oldValue = self.readPin()
        self.ticks = 0
        self.direction = Constants.FORWARD

    def isStopped(self):
        # je detekované že (asi) stojíme?
        return self.__speedTicks.isStopped

    def readPin(self):
        # přečti hodnotu pin-u z enkoderu
        return self.__pin.read_digital()

    def nextTick(self, value):
        # vyreš další tik (přičtení/odečtení)
        if self.direction == Constants.FORWARD:
            self.ticks += 1
        if self.direction == Constants.BACK:
            self.ticks -= 1

    def update(self, direction):
        self.direction = direction
        newValue = self.readPin()
        if newValue != self.__oldValue:
            self.nextTick(newValue)
            self.__oldValue = newValue
        self.__speedTicks.update(self.ticks)

    def getSpeed(self, unit, count=2, offset=0):
        # dej mi rychlost v požadované jednotce rychlosti
        speed = self.__speedTicks.calculate(count, offset)
        if unit == Constants.TicksPerSecond:
            return speed
        speed /= self.__ticksCount
        if unit == Constants.CirclePerSecond:
            return speed
        speed *= TwoPI
        if unit == Constants.RadianPerSecond:
            return speed
        speed *= self.__radius
        if unit == Constants.MeterPerSecond:
            return speed
        return 0


class Wheel:
    # Třída implementující motor
    def __init__(self, place, radius, calibrateFactors):
        self.__place = place
        self.__encoder = Encoder(place, TICKS_PER_CIRCLE, radius)
        self.__regulator = RegulatorP(6, 125)
        self.__calibrateFactors = calibrateFactors
        self.radius = radius
        self.speed = 0.0
        self.direction = Constants.FORWARD
        self.__pwmNoBack = 0
        self.__pwmNoForw = 0
        if place == Constants.RIGHT:
            self.__pwmNoBack = 2
            self.__pwmNoForw = 3
        elif place == Constants.LEFT:
            self.__pwmNoBack = 4
            self.__pwmNoForw = 5
        i2c.write(MOTOR_I2C_ADDR, bytes([0x00, 0x01]))
        i2c.write(MOTOR_I2C_ADDR, bytes([0xE8, 0xAA]))

    def emergencyShutdown(self):
        # bezpečnostní odstavení motorů
        self.speed = 0.0
        self.writePWM(self.__pwmNoBack, self.__pwmNoForw, 0)

    def isStopped(self):
        # je detekováno, že (asi) stojíme?
        return self.__encoder.isStopped()

    def getMinimumSpeed(self):
        # dej mi minimální doprednou rychlost kola robota z kalibrace kol
        return self.__calibrateFactors.minSpeed * self.radius

    def writePWM(self, offPwmNo, onPwmNo, pwm):
        # zapiš pwm přes i2c
        i2c.write(MOTOR_I2C_ADDR, bytes([offPwmNo, 0]))
        i2c.write(MOTOR_I2C_ADDR, bytes([onPwmNo, pwm]))
        self.__pwm = pwm

    def getPwmFromSpeed(self, speed):
        # spočti první hodnotu pwm z uhlove rychlosti kola
        if speed == 0.0:
            return 0
        pwmLoadCorrection = 30
        return (
            self.__calibrateFactors.a * speed
            + self.__calibrateFactors.b
            + pwmLoadCorrection
        )

    def calcAngularSpeedFromForwardSpeed(self, v):
        # spočti uhlovou rychlost kola v rad/s z dopredne rychlosti v m/s
        return v / self.radius

    def rideSpeed(self, v):
        # jeď touto dopřednou rychlostí kola
        self.speed = self.calcAngularSpeedFromForwardSpeed(v)
        if self.speed >= 0:
            self.direction = Constants.FORWARD
        else:
            self.direction = Constants.BACK
        pwm = self.getPwmFromSpeed(abs(self.speed))
        self.__ridePwm(pwm)

    def checkMinimumPwm(self, pwm):
        # zkontroluj minimální hodnotu pwm (podle toho jestli stojíme nebo jedeme)
        if self.speed != 0.0:
            if self.isStopped():
                minPwm = self.__calibrateFactors.minPwmWhenStopped
            else:
                minPwm = self.__calibrateFactors.minPwmInMotion
            if pwm < minPwm:
                pwm = minPwm
        return pwm

    def __ridePwm(self, pwm):
        # použij tuto hodnotu pwm
        pwm = self.checkMinimumPwm(int(pwm))
        if pwm < 0:
            return
        if pwm > 255:
            return
        if self.__pwmNoForw > 0 and self.__pwmNoBack > 0:
            if self.direction == Constants.FORWARD:
                self.writePWM(self.__pwmNoBack, self.__pwmNoForw, pwm)
            if self.direction == Constants.BACK:
                self.writePWM(self.__pwmNoForw, self.__pwmNoBack, pwm)

    def __changePwm(self, changeValue):
        # změn pwm o tuto hodnotu
        newPwm = 0
        if self.direction == Constants.FORWARD:
            newPwm = self.__pwm + changeValue
        if self.direction == Constants.BACK:
            newPwm = self.__pwm - changeValue
        if newPwm > 255:
            newPwm = 255
        if newPwm < 0:
            newPwm = 0
        return self.__ridePwm(newPwm)

    def getSpeed(self, unit, count=5, offset=0):
        # dej mi zmerenou rychlost kola v teto jednotce
        return self.__encoder.getSpeed(unit, count, offset)

    def regulatePwm(self):
        # reguluj pwm podle zmerene rychlosti kola
        time = ticks_ms()
        if self.__regulator.isTimeout(time):
            measureSpeed = self.__encoder.getSpeed(Constants.RadianPerSecond)
            changePwm = self.__regulator.getActionIntervention(
                time, self.speed, measureSpeed
            )
            self.__changePwm(changePwm)

    def update(self):
        self.__encoder.update(self.direction)
        self.regulatePwm()


class MotionControl:
    # Třída implementující kinematiku robota
    def __init__(
        self, wheelbase, wheelDiameter, velocity, calibrateLeft, calibrateRight
    ):
        self.__d = wheelbase / 2
        self.__r = wheelDiameter / 2
        self.velocity = velocity
        self.__wheelLeft = Wheel(Constants.LEFT, self.__r, calibrateLeft)
        self.__wheelRight = Wheel(Constants.RIGHT, self.__r, calibrateRight)

    def emergencyShutdown(self):
        # bezpečnostní odstavení motorů robota
        try:
            self.newVelocity(0, 0)
        except BaseException as e:
            self.__wheelLeft.emergencyShutdown()
            self.__wheelRight.emergencyShutdown()
            raise e

    def getMinimumSpeed(self):
        # dej mi minimální rychlost robota z kalibtačních hodnot
        minimumLeft = self.__wheelLeft.getMinimumSpeed()
        minimumRight = self.__wheelRight.getMinimumSpeed()
        return max(minimumLeft, minimumRight)

    def newVelocityIfChanged(self, forward, angular):
        if (self.velocity.forward != forward) or (self.velocity.angular != angular):
            self.newVelocity(forward, angular)

    #   print(forward, angular)

    def newVelocity(self, forward, angular):
        # nastav nové požadované rychlosti pohybu robota a přepočti je podle kinematiky do jednotlivých motorů
        self.velocity.forward = forward
        self.velocity.angular = angular
        # kinematika diferencionalniho podvozku
        self.__wheelLeft.rideSpeed(forward - self.__d * angular)
        self.__wheelRight.rideSpeed(forward + self.__d * angular)

    def update(self):
        self.__wheelLeft.update()
        self.__wheelRight.update()

class Robot:
    # Základní třída s robotem
    def __init__(self, leftCalibrate, rightCalibrate):
        self.state = Constants.ST_Start
        self.stateExecComm = Constants.ST_Start
        velocity = Velocity()
        i2c.init(freq=400_000)
        self.__senzors = Senzors()
#        self.__sonar = Sonar()
        self.__regulatorDistance = RegulatorP(0.5, 500)
        self.__regulatorTurn = RegulatorP(0.5, 50)
#        self.__lightsControl = LightsControl(velocity)
        self.motionControl = MotionControl(
            0.15, 0.067, velocity, leftCalibrate, rightCalibrate
        )
        self.motionControl.newVelocity(0, 0)
        #        self.__maxSpeed = 0.3
        #        self.__minSpeed = self.motionControl.getMinimumSpeed()
        self.__controlTurnTime = ticks_ms()
        self.__turnLastAngular = 0
        self.__timeTurn = 0
        self.__timeTurnStop = 0
        self.displayText("X")

    def emergencyShutdown(self):
        # bezpečnostní zastavení robota (něco špatného se stalo)
        self.motionControl.emergencyShutdown()
        self.displayText("S")

    def supplyVoltage(self):
        # vrať velikost napájecího napětí
        return 0.00898 * pin2.read_analog()

    def displayText(self, text):
        display.show(text, delay=0, wait=False)
        print(text)

#    def getObstacleDistance(self):
#        # vrať poslední známou vzdálenost od překážky
#        return self.__sonar.averageDistance

    def testBumber(self):
        # pokud jsme narazili tak zastav
        if self.__senzors.getSenzor(Senzors.ObstaleLeft) or self.__senzors.getSenzor(Senzors.ObstaleRight):
            self.motionControl.newVelocity(0, 0)

    def speedLimitation(self, speed):
        # omezení maximální a minimální dopředné rychlosti
        if speed >= 0:
            sign = 1
        else:
            sign = -1
        absSpeed = abs(speed)
        if absSpeed > self.__maxSpeed:
            absSpeed = self.__maxSpeed
        elif absSpeed < self.__minSpeed:
            absSpeed = self.__minSpeed
        return sign * absSpeed

    def acceptableDistance(self, regulatedDistance, distance):
        # jsme v přijatelné vzdálenosti od překážky?
        distanceDif = abs(distance - regulatedDistance)
        return distanceDif <= 0.03

    def controlSpeed(self):
        # reguluj dopředou rychlost podle vzdálenosti od překážky
        time = ticks_ms()
        if self.__regulatorDistance.isTimeout(time):
            regulatedDistance = 0.2
            distance = self.getObstacleDistance()
            if self.acceptableDistance(regulatedDistance, distance):
                newSpeed = 0
            else:
                newSpeed = self.__regulatorDistance.getActionIntervention(
                    time, -regulatedDistance, -distance
                )
                newSpeed = self.speedLimitation(newSpeed)
            self.motionControl.newVelocity(newSpeed, 0)

    def getTurnCoef(self):
        x = self.__timeTurn - 1
        return x * x * 0.08 + x * 0.45 + 1.0

    def conditionChangeTurn(self, direction, back):
#        self.displayText(direction)
        # predpokladam ze jsem na care tak pojedu rychleji a nebudu zatacet
        forward = 0.10
        angular = 0
        if direction == "0":
            # pokud jsem mimo caru jed pomaleji ale zatacej stejne jako posledne
            forward = 0.05
            angular = self.__turnLastAngular
        else:
            self.__timeTurnStop = 0
            if direction == "L":
                # vidim caru pod levym senzorem, zpomal a zatoc doleva
                forward = 0.05
                angular = 0.02 * self.getTurnCoef()
            if direction == "R":
                # vidim caru pod pravym senzorem, zpomal a zatoc doprava
                self.__directionTurn = Constants.RIGHT
                forward = 0.05
                angular = -0.02 * self.getTurnCoef()
        # pokud se požadovaná úhlová rychlost změnila, požádáme ovladač motorů o změnu rychlosti
        self.__turnLastAngular = angular
        if back:
            forward *= -1
        self.motionControl.newVelocityIfChanged(forward, angular)

    def rideLine(self, back):
        # reguluj zatáčení podle sledovače čáry
        time = ticks_ms()
        if self.__regulatorTurn.isTimeout(time):
            self.__controlTurnTime = time
            if not self.__senzors.getSenzor(Senzors.LineTrackMiddle):
                self.conditionChangeTurn("C", back)
                self.__timeTurn = 0
            else:
                if not self.__senzors.getSenzor(Senzors.LineTrackLeft):
                    self.__timeTurn += 1
                    self.conditionChangeTurn("L", back)
                elif not self.__senzors.getSenzor(Senzors.LineTrackRight):
                    self.__timeTurn += 1
                    self.conditionChangeTurn("R", back)
                else:
                    self.conditionChangeTurn("0", back)

    def update(self):
        self.motionControl.update()
        self.__senzors.update()
#        self.__sonar.update()
#        self.__lightsControl.update()
        self.testBumber()

    def exitCrossRoads(self, direction):
        angular = 0
        forward = 0.1
        if direction == Constants.BACK:
            forward *= -1
        if direction == Constants.LEFT:
            angular = 0.1
        if direction == Constants.RIGHT:
            angular = -0.1
        self.motionControl.newVelocityIfChanged(forward, angular)

        if self.__senzors.getSituationLine()==Constants.CrossRoads:
            self.__timeExitCRstart = ticks_ms()
            return False

        if (direction==Constants.LEFT) or (direction==Constants.RIGHT):
            # skoncime s popojizdenim az za chvili (abychom byli stredem otaceni nad krizovatkou)
            if ticks_diff(ticks_ms(), self.__timeExitCRstart) < 300:
                return False
        return True

    def turningToLine(self, direction, angularSpeed):
        if direction==Constants.LEFT:
            angular = angularSpeed
        if direction==Constants.RIGHT:
            angular = -angularSpeed
        self.motionControl.newVelocityIfChanged(0, angular)
        # muzeme skoncit otaceni jen kdyz trva alespon nejakou dobu (prevence chyceni cary pred sebou)
        if ticks_diff(ticks_ms(), self.__timeTurnStart) > 1000:
            if not self.__senzors.getSenzor(Senzors.LineTrackMiddle):
                return True
        return False

    #FIXME: Toto by asi melo byt nekde jinde nez ve tride Robot
    def executeCommand(self, command):
        if (self.stateExecComm == Constants.ST_Success) or (self.stateExecComm == Constants.ST_Failure):
            self.stateExecComm = Constants.ST_Start
            self.displayText("s")

        if self.stateExecComm == Constants.ST_Start:
            # zatim nemame nic co by jsme tu delali
            self.stateExecComm = Constants.ST_EC_ExitCrossRoads
            self.stateExecComm = Constants.ST_EC_ExitCrossRoads
            self.displayText("e")

        if self.stateExecComm == Constants.ST_EC_ExitCrossRoads:
            # Popojed nebo couvni tak, aby jsi opustil krizovatku. A delej to tak dlouho dokud to neni hotove
            isDone = self.exitCrossRoads(command)
            if isDone:
                if (command==Constants.LEFT) or (command==Constants.RIGHT):
                    # budeme se tocit
                    self.stateExecComm = Constants.ST_EC_Turn
                    self.displayText("t")
                else:
                    self.stateExecComm = Constants.ST_Success
                    self.displayText("o")
                    return True

        if self.stateExecComm == Constants.ST_EC_Turn:
            # zapamatujeme si kdy jsme zacali zatacet
            self.__timeTurnStart = ticks_ms()
            self.stateExecComm = Constants.ST_EC_TurnToMiddleSenzor
            self.displayText("m")

        if self.stateExecComm == Constants.ST_EC_TurnToMiddleSenzor:
            isDone = self.turningToLine(command, 0.3)
            if isDone:
                self.stateExecComm = Constants.ST_Success
                self.displayText("o")
                return True

        return False

    #FIXME: Toto by asi melo byt nekde jinde nez ve tride Robot
    def stateMachine(self):
        nextCommand = Constants.LEFT
        actualCommand = Constants.FORWARD
        back = False

        if self.state == Constants.ST_Start:
            self.displayText("S")
            # nemame tu co delat, jdeme si zajezdit po care
            self.state = Constants.ST_RidingLine

        if self.state == Constants.ST_RidingLine:
            self.rideLine(actualCommand == Constants.BACK)
            # zkontroluj jestli nejsme na krizovatce nebo jestli jsme se neztratili
            situation = self.__senzors.getSituationLine()
            if situation == Constants.CrossRoads:
                self.state = Constants.ST_ExecuteCommand
                self.displayText("D")

            elif situation == Constants.NONE:
                self.state = Constants.ST_Failure
            # jinak zustavame a jedeme po care dal

        if self.state == Constants.ST_ExecuteCommand:
            # zpracuj prikaz na krizovatce a delej to tak dlouho dokud to neni hotove
            isDone = self.executeCommand(nextCommand)
            if isDone:
                self.state = Constants.ST_RidingLine

        if (self.state == Constants.ST_Success) or (self.state == Constants.ST_Failure):
            self.displayText("E")
            self.motionControl.newVelocity(0, 0)
            return True

        return False

def memory():
    gc.collect()
    print(gc.mem_free())

if __name__ == "__main__":
    memory()
    print("Start")
    leftCalibrate = CalibrateFactors(1.0, 110, 70, 11.692, 28.643)
    rightCalibrate = CalibrateFactors(1.0, 110, 70, 12.259, 30.332)
    robot = None
    try:
        robot = Robot(leftCalibrate, rightCalibrate)
        lastPrint = ticks_ms()
        while not button_a.was_pressed():
            robot.update()
            isEnd = robot.stateMachine()
            if isEnd:
                break
            sleep(1)
        robot.motionControl.newVelocity(0, 0)
        memory()
        print("Stop")
    except BaseException as e:
        if robot:
            robot.emergencyShutdown()
        print("Exception")
        raise e
