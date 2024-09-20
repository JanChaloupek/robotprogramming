_B=False
_A=.0
from neopixel import NeoPixel
from microbit import i2c,pin0,pin8,pin12,pin14,pin15,button_a,button_b,sleep
from utime import ticks_ms,ticks_us,ticks_diff
from machine import time_pulse_us
MOTOR_I2C_ADDR=112
TICKS_PER_CIRCLE=40
class Direction:NONE=0;LEFT=1;RIGHT=2;FORWARD=11;BACK=12
class HeadLightEnum:OFF=0;POTKAVACI=1;DALKOVA=2
class Unit:TicksPerSecond=1;CirclePerSecond=2;RadianPerSecond=3;MeterPerSecond=4
class Velocity:
	def __init__(A):A.forward=_A;A.angular=_A
class CalibrateFactors:
	def __init__(A,min_rychlost,min_pwm_rozjezd,min_pwm_dojezd,a,b):A.minSpeed=min_rychlost;A.minPwmWhenStopped=min_pwm_rozjezd;A.minPwmInMotion=min_pwm_dojezd;A.a=a;A.b=b
class RegulatorP:
	def __init__(A,p,timeout_ms):A.__k=p;A.__timeout_ms=timeout_ms;A.__lastRegulationTime=ticks_ms()
	def isTimeout(A,time):B=ticks_diff(time,A.__lastRegulationTime);return B>=A.__timeout_ms
	def getOutput(A,time,inputNominal,inputActual):A.__lastRegulationTime=time;B=inputNominal-inputActual;C=A.__k*B;return C
class Senzors:
	ObstaleRight=64;ObstaleLeft=32;LineTrackRight=16;LineTrackMiddle=8;LineTrackLeft=4
	def __init__(A):A.__timeout_ms=50;A.readData()
	def readData(A,time=0):
		A.__data=i2c.read(56,1)[0]
		if time==0:A.__lastTimeRead=ticks_ms()
		else:A.__lastTimeRead=time
	def getSenzor(A,senzor):return A.__data&senzor==0
	def isTimeout(A,time):B=ticks_diff(time,A.__lastTimeRead);return B>=A.__timeout_ms
	def update(A):
		B=ticks_ms()
		if A.isTimeout(B):A.readData(B)
class IndicatorState:
	NONE=0;SPACE=1;LIGHT=2
	def __init__(A):A.reset()
	def reset(A):A.set(A.NONE)
	def set(A,value):A.value=value;A.start=ticks_ms()
	def isDiferent(A,other):return A.value!=other
	def timeout(A):return ticks_diff(ticks_ms(),A.start)>400
	def change(A):A.set(A.SPACE if A.value==A.LIGHT else A.LIGHT)
	def update(A):
		if A.value!=A.NONE:
			if A.timeout():A.change()
		else:A.set(A.LIGHT)
class Lights:
	color_led_off=0,0,0;color_led_orange=100,35,0;color_led_white=60,60,60;color_led_white_hi=255,255,255;color_led_red=60,0,0;color_led_red_br=255,0,0
	def __init__(A):A.__np=NeoPixel(pin0,8);A.__writeTime=0
	def setColor(A,ledNo,color):A.__np[ledNo]=color
	def setColorToLedList(A,ledList,color):
		for B in ledList:A.setColor(B,color)
	def isTimeout(A):return ticks_diff(ticks_ms(),A.__writeTime)>100
	def write(A):A.__np.write();A.__writeTime=ticks_ms()
class LightsControl:
	ind_all=1,2,4,7;ind_left=1,4;ind_right=2,7;head_lights=0,3;back_lights=5,6;inside_light=0,3,5,6;reverse_lights=5,
	def __init__(A,velocity):A.__lights=Lights();A.__indState=IndicatorState();A.__velocity=velocity;A.setMain(HeadLightEnum.POTKAVACI);A.setDirectionFromVelocity();A.setReverseFromVelocity();A.__isBrake=_B;A.__brakeTime=0;A.warning=_B
	def setMain(A,value):A.__main=value
	def setDirectionFromVelocity(A):
		if A.__velocity.angular>_A:A.__indDirection=Direction.LEFT
		elif A.__velocity.angular<_A:A.__indDirection=Direction.RIGHT
		else:A.__indDirection=Direction.NONE
	def setReverseFromVelocity(A):A.__isReverse=A.__velocity.forward<_A
	def setBrakeFromVelocity(A):
		if _B:A.__isBrake=True;A.__brakeTime=ticks_ms()
	def isBrake(A):
		if A.__isBrake:
			B=ticks_diff(ticks_ms(),A.__brakeTime)
			if B<1000:return True
			A.__isBrake=_B
		return _B
	def update(A):
		A.setDirectionFromVelocity();A.setReverseFromVelocity();A.setBrakeFromVelocity();D=A.__indState.value
		if A.__indDirection!=Direction.NONE or A.warning:A.__indState.update()
		else:A.__indState.reset()
		if A.__indState.isDiferent(D)or A.__lights.isTimeout():
			if A.__indState.value==IndicatorState.LIGHT:
				if A.__direction==Direction.LEFT or A.warning:A.__lights.setColorToLedList(A.ind_left,Lights.color_led_orange)
				if A.__direction==Direction.RIGHT or A.warning:A.__lights.setColorToLedList(A.ind_right,Lights.color_led_orange)
			else:A.__lights.setColorToLedList(A.ind_all,Lights.color_led_off)
			C=Lights.color_led_off;B=Lights.color_led_off
			if A.__main==HeadLightEnum.POTKAVACI:C=Lights.color_led_white;B=Lights.color_led_red
			if A.__main==HeadLightEnum.DALKOVA:C=Lights.color_led_white_hi;B=Lights.color_led_red
			if A.isBrake():B=Lights.color_led_red_br
			A.__lights.setColorToLedList(A.head_lights,C);A.__lights.setColorToLedList(A.back_lights,B)
			if A.__isReverse:A.__lights.setColorToLedList(A.reverse_lights,Lights.color_led_white)
			A.__lights.write()
class SpeedTicks:
	LIMIT=50
	def __init__(A):A.__index=-1;A.__times=[0]*A.LIMIT;A.__ticks=[0]*A.LIMIT;A.__countValues=-1;A.__lastTime=-1;A.isStopped=True
	def getNewIndex(A,time):
		B=int(time/100000)
		if B==A.__lastTime:return-1
		else:A.__lastTime=B;return(A.__index+1)%A.LIMIT
	def nextValues(A,newIndex,time,ticks):
		C=ticks;B=newIndex
		if A.__countValues<A.LIMIT:
			A.__countValues+=1
			if A.__countValues>2:A.isStopped=A.__ticks[A.__index]-C==0
		A.__times[B]=time;A.__ticks[B]=C;A.__index=B
	def update(A,ticks):
		B=ticks_us();C=A.getNewIndex(B)
		if C>=0:A.nextValues(C,B,ticks)
	def calculate(B,count=5,offset=0):
		C=offset;A=count
		if A<2:A=10
		if A+C>=B.__countValues:A=B.__countValues-C-1
		if A<2:return 0
		D=B.__calculate(A,C);E=B.__calculate(A,C+1);return(D+E)/2
	def __calculate(A,count,offset):B=(A.__index-offset)%A.LIMIT;C=(B-count+1)%A.LIMIT;D=A.__times[B]-A.__times[C];E=A.__ticks[B]-A.__ticks[C];return 1000000*E/D
class Encoder:
	def __init__(A,place):
		if place==Direction.LEFT:A.__pin=pin14
		else:A.__pin=pin15
		A.__speedTicks=SpeedTicks();A.__oldValue=A.readPin();A.ticks=0;A.direction=Direction.FORWARD
	def isStopped(A):return A.__speedTicks.isStopped
	def readPin(A):return A.__pin.read_digital()
	def nextTick(A):
		if A.direction==Direction.FORWARD:A.ticks+=1;return 0
		if A.direction==Direction.BACK:A.ticks-=1;return 0
		return-1
	def update(A,direction):
		A.direction=direction;B=A.readPin()
		if B!=A.__oldValue:A.nextTick();A.__oldValue=B
		A.__speedTicks.update(A.ticks)
	def getSpeed(C,unit,count=5,offset=0):
		B=unit;A=C.__speedTicks.calculate(count,offset)
		if B==Unit.TicksPerSecond:return A
		A/=TICKS_PER_CIRCLE
		if B==Unit.CirclePerSecond:return A
		A*=6.2832
		if B==Unit.RadianPerSecond:return A
		return 0
class Wheel:
	def __init__(A,place,radius,calibrateFactors):
		B=place;A.__place=B;A.__encoder=Encoder(B);A.__regulator=RegulatorP(6,500);A.__calibrateFactors=calibrateFactors;A.radius=radius;A.speed=_A;A.direction=Direction.FORWARD
		if B==Direction.RIGHT:A.__pwmNoBack=2;A.__pwmNoForw=3
		elif B==Direction.LEFT:A.__pwmNoBack=4;A.__pwmNoForw=5
		else:A.__pwmNoBack=0;A.__pwmNoForw=0
		i2c.write(MOTOR_I2C_ADDR,bytes([0,1]));i2c.write(MOTOR_I2C_ADDR,bytes([232,170]))
	def emergencyShutdown(A):A.speed=_A;A.writePWM(A.__pwmNoBack,A.__pwmNoForw,0)
	def isStopped(A):return A.__encoder.isStopped()
	def getMinimumSpeed(A):return A.__calibrateFactors.minSpeed
	def writePWM(A,offPwmNo,onPwmNo,pwm):i2c.write(MOTOR_I2C_ADDR,bytes([offPwmNo,0]));i2c.write(MOTOR_I2C_ADDR,bytes([onPwmNo,pwm]));A.__pwm=pwm
	def getPwmFromSpeed(A,speed):
		B=speed
		if B==_A:return 0
		return A.__calibrateFactors.a*B+A.__calibrateFactors.b
	def rideSpeed(A,speed):
		B=speed;A.speed=B
		if A.speed>=0:A.direction=Direction.FORWARD
		else:A.direction=Direction.BACK
		C=A.getPwmFromSpeed(abs(B));A.__ridePwm(C)
	def checkMinimumPwm(A,pwm):
		B=pwm
		if A.speed!=_A:
			if A.isStopped():C=A.__calibrateFactors.minPwmWhenStopped
			else:C=A.__calibrateFactors.minPwmInMotion
			if B<C:B=C
		return B
	def __ridePwm(A,pwm):
		B=pwm;C=B;B=int(B);B=A.checkMinimumPwm(B)
		if B<0:return
		if B>255:return
		if A.__pwmNoForw>0 and A.__pwmNoBack>0:
			if A.direction==Direction.FORWARD:return A.writePWM(A.__pwmNoBack,A.__pwmNoForw,B)
			if A.direction==Direction.BACK:return A.writePWM(A.__pwmNoForw,A.__pwmNoBack,B)
			return
	def __changePwm(B,changeValue):
		C=changeValue;A=0
		if B.direction==Direction.FORWARD:A=B.__pwm+C
		if B.direction==Direction.BACK:A=B.__pwm-C
		if A>255:A=255
		if A<0:A=0
		return B.__ridePwm(A)
	def getSpeed(A,unit,count=5,offset=0):
		C=offset;B=count
		if unit==Unit.MeterPerSecond:return A.radius*A.__encoder.getSpeed(Unit.RadianPerSecond,B,C)
		return A.__encoder.getSpeed(unit,B,C)
	def regulate(A):
		B=ticks_ms()
		if A.__regulator.isTimeout(B):C=A.__encoder.getSpeed(Unit.RadianPerSecond);D=A.__regulator.getOutput(B,A.speed,C);A.__changePwm(D)
	def update(A):A.__encoder.update(A.direction);A.regulate()
class MotionControl:
	def __init__(A,wheelbase,wheelDiameter,velocity,calibrateLeft,calibrateRight):A.__d=wheelbase/2;A.__r=wheelDiameter/2;A.velocity=velocity;A.__wheelLeft=Wheel(Direction.LEFT,A.__r,calibrateLeft);A.__wheelRight=Wheel(Direction.RIGHT,A.__r,calibrateRight)
	def emergencyShutdown(A):
		try:A.newVelocity(0,0)
		except BaseException as B:A.__wheelLeft.emergencyShutdown();A.__wheelRight.emergencyShutdown();raise B
	def getMinimumSpeed(A):B=A.__wheelLeft.getMinimumSpeed();C=A.__wheelRight.getMinimumSpeed();return max(B,C)
	def newVelocity(A,forward,angular):A.velocity.forward=forward;A.velocity.angular=angular;A.__wheelLeft.rideSpeed(A.velocity.forward-A.__d*A.velocity.angular);A.__wheelRight.rideSpeed(A.velocity.forward+A.__d*A.velocity.angular)
	def calcForwardSpeedFromSpeed(A,speed):return speed/A.__r
	def update(A):A.__wheelLeft.update();A.__wheelRight.update()
class Sonar:
	MAX_DISTANCE=10
	def __init__(A,timeoutMeasure):A.__trigger=pin8;A.__trigger.write_digital(0);A.__echo=pin12;A.__echo.read_digital();A.__lastMeasureTime=0;A.__lastReturned=-3;A.__timeout_ms=timeoutMeasure;A.lastDistance=-1
	def calculateDistance(A,time):
		A.__lastMeasureTime=time;A.__trigger.write_digital(1);A.__trigger.write_digital(0);C=340;B=time_pulse_us(A.__echo,1,5000)
		if B<0:return B
		D=B/1000000;E=D*C/2;return E
	def isTimeout(A,time):B=ticks_diff(time,A.__lastMeasureTime);return B>=A.__timeout_ms
	def update(A):
		B=ticks_ms()
		if A.isTimeout(B):
			A.__lastReturned=A.calculateDistance(B)
			if A.__lastReturned>0:A.lastDistance=A.__lastReturned
			if A.__lastReturned==-1:A.lastDistance=A.MAX_DISTANCE
class Robot:
	def __init__(A,leftCalibrate,rightCalibrate):B=Velocity();i2c.init(freq=400000);A.__senzors=Senzors();A.__sonar=Sonar(300);A.__regulator=RegulatorP(15,1000);A.__lightsControl=LightsControl(B);A.motionControl=MotionControl(.15,.067,B,leftCalibrate,rightCalibrate);A.motionControl.newVelocity(0,0);A.__maxSpeed=A.motionControl.calcForwardSpeedFromSpeed(.3);A.__minSpeed=A.motionControl.getMinimumSpeed()
	def emergencyShutdown(A):A.motionControl.emergencyShutdown()
	def supplyVoltage(A):return .00898*pin2.read_analog()
	def getObstacleDistance(A):return A.__sonar.lastDistance
	def testBumber(A):
		if A.__senzors.getSenzor(Senzors.ObstaleLeft)or A.__senzors.getSenzor(Senzors.ObstaleRight):A.motionControl.newVelocity(0,0)
	def speedLimitation(B,speed):
		C=speed
		if C>=0:D=1
		else:D=-1
		A=abs(C)
		if A>B.__maxSpeed:A=B.__maxSpeed
		elif A<B.__minSpeed:A=B.__minSpeed
		return D*A
	def acceptableDistance(B,regulatedDistance,distance):A=abs(distance-regulatedDistance);return A<=.03
	def regulateSpeed(A):
		C=ticks_ms()
		if A.__regulator.isTimeout(C):
			D=.2;E=A.getObstacleDistance()
			if A.acceptableDistance(D,E):B=0
			else:B=A.__regulator.getOutput(C,-D,-E);B=A.speedLimitation(B)
			A.motionControl.newVelocity(B,0)
	def update(A):A.motionControl.update();A.__senzors.update();A.__sonar.update();A.__lightsControl.update();A.testBumber();A.regulateSpeed()
if __name__=='__main__':
	leftCalibrate=CalibrateFactors(2.8,110,75,11.692,28.643);rightCalibrate=CalibrateFactors(2.8,110,75,12.259,30.332);robot=Robot(leftCalibrate,rightCalibrate)
	try:
		while not button_a.was_pressed():robot.update();sleep(1)
		robot.motionControl.newVelocity(0,0)
	except BaseException as e:robot.emergencyShutdown();raise e