from microbit import i2c, pin8, pin12, pin14, pin15, button_a, button_b, sleep
from utime import ticks_ms, ticks_us, ticks_diff
from machine import time_pulse_us

I2C_ADDR_PWM = 0x70
TICK_PER_CIRCLE = 40

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
        self.min_speed = min_rychlost
        self.min_pwm_up = min_pwm_rozjezd
        self.min_pwm_down = min_pwm_dojezd
        self.a = a - 4.3
        self.b = b - 2

    def getPwmFromSpeed(self, speed):
        return self.a * speed + self.b

class SpeedTicks:
    # pocet desetin pro ktere si pamatujeme hodnoty
    LIMIT = 50
    def __init__(self):
        self.__index = -1        # prvni hodnota bude ulozena do indexu 0 (coz je -1 + 1)
        self.__times = [0] * self.LIMIT
        self.__ticks = [0] * self.LIMIT
        self.__countValues = -1  # ani prvni hodnotu nechci brat jako správnou (bude ignorovana)
        self.__lastTime = -1

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
        self.__times[newIndex] = time
        self.__ticks[newIndex] = ticks
        self.__index = newIndex

    # Nech nas pracovat. Musime volat dostatecne casto (idealne jednou za setinu sekundy)
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
        # zmenseni chyby, pokud nastane na krajnich hodnotach => zprumerujeme rychlost spoctenou z o 1 setinu starsimi daty
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

#        self.__perioda_rychlosti = 1_000_000
#        self.__cas_posledni_rychlosti = 0
#        self.__radiany_za_sekundu = 0

#    def vypocti_rychlost(self):
#        cas_ted = ticks_us()
#        interval_us = ticks_diff(cas_ted, self.__cas_posledni_rychlosti)
#        if interval_us >= self.__perioda_rychlosti:
#            interval_s = interval_us / 1_000_000
#            otacky = self.ticks/40
#            radiany = otacky * 6.28
#            self.__radiany_za_sekundu = radiany / interval_s
#            self.ticks = 0
#            self.__cas_posledni_rychlosti = cas_ted
#        return self.__radiany_za_sekundu

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

    def getSpeed(self, unit):
        # nejpre spocti rychlost v tikach za sekundu
        speed = self.__speedTicks.calculate()
        if unit == UnitEnum.TicksPerSecond:
            return speed
        # uprav rychlost na otacky za sekundu
        speed /= TICK_PER_CIRCLE
        if unit == UnitEnum.CirclePerSecond:
            return speed
        # uprav rychlost na radiany za sekundu
        speed *= 6.28
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
        return  diff > self.__timeout_ms

    def getOutput(self, time, inputNominal, inputActual):
        self.__lastRegulationTime = time
        error = inputNominal - inputActual
        changeValue = self.__k * error
#        print("regulator.getOutput - input:", inputNominal, inputActual, error, changeValue)
        return changeValue

class Senzors:
    ObstaleRight = 0x40
    ObstaleLeft = 0x20

    def __init__(self):
        self.__lastTimeRead = 0
        self.__timeout_ms = 250
        self.__data = 0

    def readNewData(self, time=0):
        self.__data = i2c.read(0x38, 1)
        if time == 0:
            self.__lastTimeRead = ticks_ms()
        else:
            self.__lastTimeRead = time

    def getSenzor(self, senzor):
        return (self.__data[0] & senzor) == 0

    def isTimeout(self, time):
        diff = ticks_diff(time, self.__lastTimeRead)
        return  diff > self.__timeout_ms

    def update(self):
        time = ticks_ms()
        if self.isTimeout(time):
            self.readNewData(time)


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
        i2c.write(I2C_ADDR_PWM, bytes([0x00, 0x01]))
        i2c.write(I2C_ADDR_PWM, bytes([0xE8, 0xAA]))

    def writePWM(self, offPwmNo, onPwmNo, pwm):
        i2c.write(I2C_ADDR_PWM, bytes([offPwmNo, 0]))
        i2c.write(I2C_ADDR_PWM, bytes([onPwmNo, pwm]))
        self.__pwm = pwm
#        print("Wheel.writePWM:", pwm)
        return 0

    def rideSpeed(self, speed):
        self.speed = speed
        if self.speed >= 0:
            self.direction = DirectionEnum.FORWARD
        else:
            self.direction = DirectionEnum.BACK
        pwm = self.__calibrateFactors.getPwmFromSpeed(abs(speed))
#        print("wheel.rideSpeed:", self.speed, pwm, self.direction)
        self.__ridePwm(pwm)

    def __ridePwm(self, pwm):
        pwm = int(pwm)
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

    def getSpeed(self, unit):
        if unit == UnitEnum.MeterPerSecond:
            return self.radius * self.__encoder.getSpeed(UnitEnum.RadianPerSecond)
        return self.__encoder.getSpeed(unit)

    def regulate(self):
        time = ticks_ms()
        if self.__regulator.isTimeout(time):
            measureSpeed = self.__encoder.getSpeed(UnitEnum.RadianPerSecond)
#            measureSpeed = self.__encoder.vypocti_rychlost()
#            print("Wheel.regulate - speed:", measureSpeed, self.speed)

            changeValue = self.__regulator.getOutput(time, self.speed, measureSpeed)
#            print("Wheel.regulate . changePwm", changeValue)
            self.__changePwm(changeValue)

    def convertWhellDirectionToEncoder(self):
        if self.direction == DirectionEnum.BACK:
            return DirectionEnum.DOWN
        return DirectionEnum.UP

    def update(self):
        self.__encoder.update(self.convertWhellDirectionToEncoder())
        self.regulate()

class MotionControl:
    def __init__(self, wheelbase, wheelDiameter, calibrateLeft, calibrateRight):
        self.__d = wheelbase / 2
        self.velocity = Velocity()
        self.__wheelLeft = Wheel(DirectionEnum.LEFT, wheelDiameter / 2, calibrateLeft)
        self.__wheelRight = Wheel(DirectionEnum.RIGHT, wheelDiameter / 2,  calibrateRight)
        self.__speedLeft = 0.0
        self.__speedRight = 0.0

    def newVelocity(self, forward, angular):
        self.velocity.forward = forward
        self.velocity.angular = angular
        self.__wheelLeft.rideSpeed(self.velocity.forward - self.__d * self.velocity.angular)
        self.__wheelRight.rideSpeed(self.velocity.forward + self.__d * self.velocity.angular)

    def update(self):
        self.__wheelLeft.update()
        self.__wheelRight.update()

class Sonar:
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
        return  diff > self.__timeout_ms

    def update(self):
        time = ticks_ms()
        if self.isTimeout(time):
            self.__lastReturned = self.calculateDistance(time)
            if self.__lastReturned > 0:
                self.lastDistance = self.__lastReturned

class Robot:
    def __init__(self):
        i2c.init(freq=400_000)
        self.__senzors = Senzors()
        self.__sonar = Sonar(300)
        self.__regulator = RegulatorP(15, 1_000)
        left = CalibrateFactors(1.861183, 50, 39, 11.4267317247637, 22.6641139347994)
        right = CalibrateFactors(2.017688, 50, 37, 12.0986865665863, 24.2334514165142)
        self.motionControl = MotionControl(0.15, 0.067, left, right)
        self.motionControl.newVelocity(1, 0)
        self.counterUpdate = 0

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
        if absSpeed > 7:
            absSpeed = 7
        elif absSpeed < 0.5:
            absSpeed = 0
        elif absSpeed < 2.5:
            absSpeed = 2.5
        return sign * absSpeed

    def regulateSpeed(self):
        time = ticks_ms()
        if self.__regulator.isTimeout(time):
            self.counterUpdate += 1
            distance = self.getObstacleDistance()
            print(distance)
            newSpeedForward = self.__regulator.getOutput(time, -0.2, -distance)
            newSpeedLimit = self.speedLimitation(newSpeedForward)
            print("regualteSpeed:", distance, newSpeedForward, newSpeedLimit)
            self.motionControl.newVelocity(newSpeedLimit, 0)

    def update(self):
        self.motionControl.update()
        self.__senzors.update()
        self.__sonar.update()
        self.testBumber()
        self.regulateSpeed()

def main():
    robot = Robot()
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
            print(speedL, speedR, robot.__sonar.lastDistance, robot.counterUpdate)
        sleep(1)
    robot.motionControl.newVelocity(0, 0)

if __name__ == "__main__":
    main()
