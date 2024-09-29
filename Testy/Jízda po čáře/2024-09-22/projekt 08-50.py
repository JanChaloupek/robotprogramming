from neopixel import NeoPixel
from microbit import i2c, pin0, pin8, pin12, pin14, pin15, button_a, button_b, sleep, display
from utime import ticks_ms, ticks_us, ticks_diff, ticks_add
from machine import time_pulse_us

MOTOR_I2C_ADDR = 0x70
TICKS_PER_CIRCLE = 40

class Direction:
    NONE = 0
    LEFT = 1
    RIGHT = 2
    FORWARD = 11
    BACK = 12

class HeadLightEnum:
    OFF = 0
    POTKAVACI = 1
    DALKOVA = 2

class Unit:
    # jednotky rychlosti
    TicksPerSecond = 1
    CirclePerSecond = 2
    RadianPerSecond = 3
    MeterPerSecond = 4

class Velocity:
    # třída pro uložení požadované rychlosti robota
    def __init__(self):
        self.forward = 0.0
        self.angular = 0.0

class CalibrateFactors:
    # třída pro uložení kalibračních hodnot pro motor
    def __init__(self, min_rychlost, min_pwm_rozjezd, min_pwm_dojezd, a, b):
        self.minSpeed = min_rychlost
        self.minPwmWhenStopped = min_pwm_rozjezd
        self.minPwmInMotion = min_pwm_dojezd
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
        return  diff >= self.__timeout_ms

    def getOutput(self, time, inputNominal, inputActual):
        # vypočti akční zásah
        self.__lastRegulationTime = time
        error = inputNominal - inputActual
        changeValue = self.__k * error
        return changeValue

class Senzors:
    # třída vyčítající senzory po i2c a získání jejich stavu dotazem na konkretni z nich
    ObstaleRight = 0x40
    ObstaleLeft = 0x20
    LineTrackRight = 0x10
    LineTrackMiddle = 0x08
    LineTrackLeft = 0x04

    def __init__(self):
        self.__timeout_ms = 50
        self.readData()

    def readData(self, time=0):
        # přečti data po i2c
        self.__data = i2c.read(0x38, 1)[0]
        if time == 0:
            self.__lastTimeRead = ticks_ms()
        else:
            self.__lastTimeRead = time

    def getSenzor(self, senzor):
        # vrat stav jednoho senzoru z vyčtených dat
        return (self.__data & senzor) == 0

    def isTimeout(self, time):
        # už je čas znovu vyčíst data senzoru?
        diff = ticks_diff(time, self.__lastTimeRead)
        return  diff >= self.__timeout_ms

    def update(self):
        time = ticks_ms()
        if self.isTimeout(time):
            self.readData(time)

class IndicatorState:
    NONE = 0
    SPACE = 1
    LIGHT = 2
    def __init__(self):
        self.reset()

    def set(self, value):
        self.value = value
        self.start = ticks_ms()

    def reset(self):
        self.set(self.NONE)

    def isDiferent(self, other):
        # je hodnota stavu rozdílná od předané?
        return self.value != other

    def timeout(self):
        # vypršel čas na změnu stavu blinkru?
        return ticks_diff(ticks_ms(), self.start) > 400

    def change(self):
        # změn stav blinkru
        self.set(self.SPACE if self.value == self.LIGHT else self.LIGHT)

    def update(self):
        if self.value != self.NONE:
            if self.timeout():
                self.change()
        else:
            self.set(self.LIGHT)

class Lights:
    # Třída implementující ledky jako světla robota (používá knihovnu NeoPixel)
    color_led_off = (0, 0, 0)
    color_led_orange = (100, 35, 0)
    color_led_white = (60, 60, 60)
    color_led_white_hi = (255, 255, 255)
    color_led_red = (60, 0, 0)
    color_led_red_br = (255, 0, 0)

    def __init__(self):
        self.__np = NeoPixel(pin0, 8)
        self.__writeTime = 0

    def setColor(self, ledNo, color):
        # nastav barvu pro jednu led-ku
        self.__np[ledNo] = color

    def setColorToLedList(self, ledList, color):
        # nastav barvu pro seznam led-ek
        for ledNo in ledList:
            self.setColor(ledNo, color)

    def isTimeout(self):
        # vypršel čas na pravidelné zapsaní barev do ledek?
        return ticks_diff(ticks_ms(), self.__writeTime) > 100

    def write(self):
        # zapiš nastavené barvy do led-ek
        self.__np.write()
        self.__writeTime = ticks_ms()

class LightsControl:
    # Třída implementující jednotlivá světla (blinkry, zpátečku, brzdy, potkávací a dálková světla)
    ind_all = (1, 2, 4, 7)
    ind_left = (1, 4)
    ind_right = (2, 7)
    head_lights = (0, 3)
    back_lights = (5, 6)
    inside_light = (0, 3, 5, 6)
    reverse_lights = (5,)
    def __init__(self, velocity):
        self.__lights = Lights()
        self.__indState = IndicatorState()
        self.__velocity = velocity
        self.setMain(HeadLightEnum.POTKAVACI)
        self.setDirectionFromVelocity()
        self.setReverseFromVelocity()
        self.__isBrake = False
        self.__brakeTime = 0
        self.warning = False

    def setMain(self, value):
        # zapni typ hlavních světel (vypnuto, potkávací, dalková)
        self.__main = value

    def setDirectionFromVelocity(self):
        # spočti směr blikání z požadované úhlové rychlosti robota
        if self.__velocity.angular > 0.0:
            self.__indDirection = Direction.LEFT
        elif self.__velocity.angular < 0.0:
            self.__indDirection = Direction.RIGHT
        else:
            self.__indDirection = Direction.NONE

    def setReverseFromVelocity(self):
        # spočti zapnutí couvacího světla z požadované dopředné rychlosti robota
        self.__isReverse = self.__velocity.forward < 0.0

    def setBrakeFromVelocity(self):
        # spočti brzdové světlo z požadované dopředné rychlosti robota
        if False:
            self.__isBrake = True
            self.__brakeTime = ticks_ms()

    def isBrake(self):
        # má svítit brzdové světlo?
        if self.__isBrake:
            diff = ticks_diff(ticks_ms(), self.__brakeTime)
            if diff < 1_000:
                return True
            self.__isBrake = False
        return False

    def update(self):
        self.setDirectionFromVelocity()
        self.setReverseFromVelocity()
        self.setBrakeFromVelocity()

        backupState = self.__indState.value
        if self.__indDirection != Direction.NONE or self.warning:
            self.__indState.update()
        else:
            self.__indState.reset()
        if self.__indState.isDiferent(backupState) or self.__lights.isTimeout():
            if self.__indState.value == IndicatorState.LIGHT:
                if self.__indDirection == Direction.LEFT or self.warning:
                    self.__lights.setColorToLedList(self.ind_left, Lights.color_led_orange)
                if self.__indDirection == Direction.RIGHT or self.warning:
                    self.__lights.setColorToLedList(self.ind_right, Lights.color_led_orange)
            else:
                self.__lights.setColorToLedList(self.ind_all, Lights.color_led_off)
            headColor = Lights.color_led_off
            backColor = Lights.color_led_off
            if self.__main == HeadLightEnum.POTKAVACI:
                headColor = Lights.color_led_white
                backColor = Lights.color_led_red
            if self.__main == HeadLightEnum.DALKOVA:
                headColor = Lights.color_led_white_hi
                backColor = Lights.color_led_red
            if self.isBrake():
                backColor = Lights.color_led_red_br
            self.__lights.setColorToLedList(self.head_lights, headColor)
            self.__lights.setColorToLedList(self.back_lights, backColor)
            if self.__isReverse:
                self.__lights.setColorToLedList(self.reverse_lights, Lights.color_led_white)
            self.__lights.write()

class SpeedTicks:
    # Třída počítající rychlost z uložené historie tiků
    LIMIT = 50
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

    def nextValues(self, newIndex, time, ticks):
        # Ulož další data do historie
        if self.__countValues < self.LIMIT:
            self.__countValues += 1
        if self.__countValues > 2:
            self.isStopped = (self.__ticks[self.__index]-ticks) == 0
        self.__times[newIndex] = time
        self.__ticks[newIndex] = ticks
        self.__index = newIndex

    def update(self, ticks):
        time = ticks_us()
        newIndex = self.getNewIndex(time)
        if newIndex >= 0:
            self.nextValues(newIndex, time, ticks)

    def calculate(self, count=5, offset=0):
        # Spočti rychlost v tikách za sekundu.
        # Použij na to count dat z historie a použij ty, které jsou offset staré
        if count < 2:
            count = 10
        if count+offset >= self.__countValues:
            count = self.__countValues - offset - 1
        if count < 2:
            return 0
        speed0 = self.__calculate(count, offset)
        speed1 = self.__calculate(count, offset+1)
        return  (speed0 + speed1) / 2

    def __calculate(self, count, offset):
        # Skutečné spočtení (bez kontrol a průměrování)
        endIndex = (self.__index - offset) % self.LIMIT
        startIndex = (endIndex - count + 1) % self.LIMIT
        diffTimes = self.__times[endIndex] - self.__times[startIndex]
        diffTicks = self.__ticks[endIndex] - self.__ticks[startIndex]
        return 1_000_000 * diffTicks / diffTimes

class Encoder:
    # Třída počítající tiky enkoderu
    def __init__(self, place):
        if place == Direction.LEFT:
            self.__pin = pin14
        else:
            self.__pin = pin15
        self.__speedTicks = SpeedTicks()
        self.__oldValue = self.readPin()
        self.ticks = 0
        self.direction = Direction.FORWARD

    def isStopped(self):
        # je detekované že (asi) stojíme?
        return self.__speedTicks.isStopped

    def readPin(self):
        # přečti hodnotu pin-u z enkoderu
        return self.__pin.read_digital()

    def nextTick(self):
        # vyreš další tik (přičtení/odečtení)
        if self.direction == Direction.FORWARD:
            self.ticks += 1
        if self.direction == Direction.BACK:
            self.ticks -= 1

    def update(self, direction):
        self.direction = direction
        newValue = self.readPin()
        if (newValue != self.__oldValue):
            self.nextTick()
            self.__oldValue = newValue
        self.__speedTicks.update(self.ticks)

    def getSpeed(self, unit, count=5, offset=0):
        # dej mi rychlost v požadované jednotce rychlosti
        speed = self.__speedTicks.calculate(count, offset)
        if unit == Unit.TicksPerSecond:
            return speed
        speed /= TICKS_PER_CIRCLE
        if unit == Unit.CirclePerSecond:
            return speed
        speed *= (2 * 3.1416)
        if unit == Unit.RadianPerSecond:
            return speed
        return 0

class Wheel:
    # Třída implementující motor
    def __init__(self, place, radius, calibrateFactors):
        self.__place = place
        self.__encoder = Encoder(place)
        self.__regulator = RegulatorP(6, 500)
        self.__calibrateFactors = calibrateFactors
        self.radius = radius
        self.speed = 0.0
        self.direction = Direction.FORWARD
        if place == Direction.RIGHT:
            self.__pwmNoBack = 2
            self.__pwmNoForw = 3
        elif place == Direction.LEFT:
            self.__pwmNoBack = 4
            self.__pwmNoForw = 5
        else:
            self.__pwmNoBack = 0
            self.__pwmNoForw = 0
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
        # dej mi minimální dopřednou rychlost robota z kalibrace kol
        return self.__calibrateFactors.minSpeed

    def writePWM(self, offPwmNo, onPwmNo, pwm):
        # zapiš pwm přes i2c
        i2c.write(MOTOR_I2C_ADDR, bytes([offPwmNo, 0]))
        i2c.write(MOTOR_I2C_ADDR, bytes([onPwmNo, pwm]))
        self.__pwm = pwm

    def getPwmFromSpeed(self, speed):
        # spočti první hodnotu pwm z dopředné rychlosti kola
        if speed==0.0:
            return 0
        return self.__calibrateFactors.a * speed + self.__calibrateFactors.b

    def rideSpeed(self, speed):
        # jeď touto dopřednou rychlostí kola
        self.speed = speed
        if self.speed >= 0:
            self.direction = Direction.FORWARD
        else:
            self.direction = Direction.BACK
        pwm = self.getPwmFromSpeed(abs(speed))
        self.__ridePwm(pwm)

    def checkMinimumPwm(self, pwm):
        # zkontroluj minimální hodnotu pwm (podle toho jestli stojíme nebo jedeme)
        if self.speed != 0.0:
            if (self.isStopped()):
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
        if self.__pwmNoForw>0 and self.__pwmNoBack>0:
            if self.direction == Direction.FORWARD:
                self.writePWM(self.__pwmNoBack, self.__pwmNoForw, pwm)
            if self.direction == Direction.BACK:
                self.writePWM(self.__pwmNoForw, self.__pwmNoBack, pwm)

    def __changePwm(self, changeValue):
        # změn pwm o tuto hodnotu
        newPwm = 0
        if self.direction == Direction.FORWARD:
            newPwm = self.__pwm + changeValue
        if self.direction == Direction.BACK:
            newPwm = self.__pwm - changeValue
        if newPwm > 255:
            newPwm = 255
        if newPwm < 0:
            newPwm  = 0
        return self.__ridePwm(newPwm)

    def getSpeed(self, unit, count=5, offset=0):
        # dej mi zmerenou rychlost kola v teto jednotce
        if unit == Unit.MeterPerSecond:
            return self.radius * self.__encoder.getSpeed(Unit.RadianPerSecond, count, offset)
        return self.__encoder.getSpeed(unit, count, offset)

    def regulate(self):
        # reguluj pwm podle zmerene rychlosti kola
        time = ticks_ms()
        if self.__regulator.isTimeout(time):
            measureSpeed = self.__encoder.getSpeed(Unit.RadianPerSecond)
            changeValue = self.__regulator.getOutput(time, self.speed, measureSpeed)
            self.__changePwm(changeValue)

    def update(self):
        self.__encoder.update(self.direction)
        self.regulate()

class MotionControl:
    # Třída implementující kinematiku robota
    def __init__(self, wheelbase, wheelDiameter, velocity, calibrateLeft, calibrateRight):
        self.__d = wheelbase / 2
        self.__r = wheelDiameter / 2
        self.velocity = velocity
        self.__wheelLeft = Wheel(Direction.LEFT, self.__r, calibrateLeft)
        self.__wheelRight = Wheel(Direction.RIGHT, self.__r,  calibrateRight)

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

    def newVelocity(self, forward, angular):
        # nastav nové pořadované rychlosti pohybu robota a přepočti je podle kinematiky do jednotlivých motorů
        self.velocity.forward = forward
        self.velocity.angular = angular
        self.__wheelLeft.rideSpeed(self.velocity.forward - self.__d * self.velocity.angular)
        self.__wheelRight.rideSpeed(self.velocity.forward + self.__d * self.velocity.angular)

    def calcForwardSpeedFromSpeed(self, speed):
        # spočti dopřednou rychlost v rad/s z rychlosti m/s
        return speed / self.__r

    def update(self):
        self.__wheelLeft.update()
        self.__wheelRight.update()

class Sonar:
    # Třída implementující měření vzdálenosti k překážce
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
        # změř a vrať vzdálenost k překážce
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
        # už je čas znovu změřit vzdálenost?
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
    # Základní třída s robotem
    def __init__(self, leftCalibrate, rightCalibrate):
        velocity = Velocity()
        i2c.init(freq=400_000)
        self.__oldTurn = 0
        self.__senzors = Senzors()
        self.__sonar = Sonar(300)
        self.__regulator = RegulatorP(15, 1_000)
        self.__lightsControl = LightsControl(velocity)
        self.motionControl = MotionControl(0.15, 0.067, velocity, leftCalibrate, rightCalibrate)
        self.motionControl.newVelocity(0, 0)
        self.__maxSpeed = self.motionControl.calcForwardSpeedFromSpeed(0.3)
        self.__minSpeed = self.motionControl.getMinimumSpeed()
        self.__turnCount = 0
        self.__controlTurnTime = ticks_ms()

    def emergencyShutdown(self):
        # bezpečnostní zastavení robota (něco špatného se stalo)
        self.motionControl.emergencyShutdown()

    def supplyVoltage(self):
        # vrať velikost napájecího napětí
        return 0.00898 * pin2.read_analog()

    def getObstacleDistance(self):
        # vrať poslední známou vzdálenost od překážky
        return self.__sonar.lastDistance

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
        distanceDif = abs(distance-regulatedDistance)
        return distanceDif <= 0.03

    def controlSpeed(self):
        # reguluj dopředou rychlost podle vzdálenosti od překážky
        time = ticks_ms()
        if self.__regulator.isTimeout(time):
            regulatedDistance = 0.2
            distance = self.getObstacleDistance()
            if self.acceptableDistance(regulatedDistance, distance):
                newSpeed = 0
            else:
                newSpeed = self.__regulator.getOutput(time, -regulatedDistance, -distance)
                newSpeed = self.speedLimitation(newSpeed)
            self.motionControl.newVelocity(newSpeed, 0)

    def conditionChangeTurn(self, turn):
        # pokud se požadovaná úhlová rychlost změnila, požádáme ovladač motorů o změnu rychlosti
        if self.__oldTurn != turn:
            self.motionControl.newVelocity(4, turn)
            self.__oldTurn = turn

    def controlTurn(self):
        # reguluj zatáčení podle sledovače čáry
        time = ticks_ms()
        if ticks_diff(time, self.__controlTurnTime) > 50:
            self.__controlTurnTime = time
            if not self.__senzors.getSenzor(Senzors.LineTrackMiddle):
                self.conditionChangeTurn(0)
                self.__turnCount = 0
            elif not self.__senzors.getSenzor(Senzors.LineTrackLeft):
                self.conditionChangeTurn(100)
                self.__turnCount = 0
            elif not self.__senzors.getSenzor(Senzors.LineTrackRight):
                self.conditionChangeTurn(-100)
                self.__turnCount = 0
            else:
                self.__turnCount += 1
                if self.__turnCount>100:
                    self.motionControl.newVelocity(0, 0)

    def update(self):
        self.motionControl.update()
        self.__senzors.update()
        self.__sonar.update()
        self.__lightsControl.update()
        self.testBumber()
#        self.controlSpeed()
        self.controlTurn()

if __name__ == "__main__":
    leftCalibrate  = CalibrateFactors(2.8, 110, 75, 11.692, 28.643)
    rightCalibrate = CalibrateFactors(2.8, 110, 75, 12.259, 30.332)
    robot = Robot(leftCalibrate, rightCalibrate)
    try:
        speed = 4
        robot.motionControl.newVelocity(speed, 0)
        lastPrint = 0
        while not button_a.was_pressed():
            robot.update()
            time = ticks_ms()
            diff = ticks_diff(time, lastPrint)
            if diff > 1_000:
                lastPrint = time
            sleep(1)
        robot.motionControl.newVelocity(0, 0)
    except BaseException as e:
        robot.emergencyShutdown()
        raise e
