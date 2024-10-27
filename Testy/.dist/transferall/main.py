_B=True
_A=False
from neopixel import NeoPixel
from microbit import i2c,pin0,pin8,pin12,pin14,pin15,button_a,button_b,sleep,display
from utime import ticks_ms,ticks_us,ticks_diff
from machine import time_pulse_us
from math import sin,cos
import gc
MOTOR_I2C_ADDR=112
TICKS_PER_CIRCLE=40
PI=3.141592
TwoPI=2*PI
HalfPI=PI/2
class Constants:NONE=0;LEFT=1;RIGHT=2;FORWARD=11;BACK=12;DippedBeams=31;HighBeams=32;TicksPerSecond=41;CirclePerSecond=42;RadianPerSecond=43;MeterPerSecond=44;Line=51;CrossRoads=52;ST_Start=100;ST_Success=101;ST_Failure=102;ST_RidingLine=111;ST_DetectCrossRoads=112;ST_LocalizeBeforeExecCommand=113;ST_ExecuteCommand=114;ST_LocalizeAfterExecCommand=116;ST_EC_ExitCrossRoads=121;ST_EC_Turn=122;ST_EC_TurnToMiddleSenzor=123
class Velocity:
	def __init__(A):A.forward=.0;A.angular=.0
class CalibrateFactors:
	def __init__(A,minSpeed,minPwmWhenStopped,minPwmInMotion,a,b):A.minSpeed=minSpeed;A.minPwmWhenStopped=minPwmWhenStopped;A.minPwmInMotion=minPwmInMotion;A.a=a;A.b=b
class RegulatorP:
	def __init__(A,p,timeout_ms):A.__k=p;A.__timeout_ms=timeout_ms;A.__lastRegulationTime=ticks_ms()
	def isTimeout(A,time):B=ticks_diff(time,A.__lastRegulationTime);return B>=A.__timeout_ms
	def getActionIntervention(A,time,inputNominal,inputActual):A.__lastRegulationTime=time;B=inputNominal-inputActual;C=A.__k*B;return C
class Senzors:
	ObstaleRight=64;ObstaleLeft=32;LineTrackRight=16;LineTrackMiddle=8;LineTrackLeft=4;LineTrackAll=28
	def __init__(A):A.__timeout_ms=50;A.__timeNotLine=0;A.__data=0;A.readData()
	def readData(A,time=0):
		A.__dataPrev=A.__data;A.__data=i2c.read(56,1)[0]^Senzors.LineTrackAll
		if time==0:A.__lastTimeRead=ticks_ms()
		else:A.__lastTimeRead=time
	def getData(A,mask):return A.__data&mask
	def getSenzor(A,senzor):return A.getData(senzor)==0
	def isTimeout(A,time):B=ticks_diff(time,A.__lastTimeRead);return B>=A.__timeout_ms
	def update(A):
		B=ticks_ms()
		if A.isTimeout(B):A.readData(B)
	def getSituationLineChar(B):
		A=0
		if B.getSenzor(Senzors.LineTrackLeft):A+=1
		if B.getSenzor(Senzors.LineTrackMiddle):A+=2
		if B.getSenzor(Senzors.LineTrackRight):A+=4
		return str(A)
	def getSituationLine(A):
		B=0
		if A.getSenzor(Senzors.LineTrackLeft):B+=1
		if A.getSenzor(Senzors.LineTrackMiddle):B+=1
		if A.getSenzor(Senzors.LineTrackRight):B+=1
		if B>=2:A.__timeNotLine=0;return Constants.CrossRoads
		if B==1:A.__timeNotLine=0;return Constants.Line
		if A.__timeNotLine==0:A.__timeNotLine=ticks_ms()
		if ticks_diff(ticks_ms(),A.__timeNotLine)<=2000:return Constants.Line
		return Constants.NONE
class LightsControl:
	color_led_off=0,0,0;color_led_orange=100,35,0;color_led_white=60,60,60;color_led_red=60,0,0
	def __init__(A,velocity):A.__velocity=velocity;A.__np=NeoPixel(pin0,8);A.direction=Constants.NONE;A.__timeout_ms=400;A.__lastTime=ticks_ms();A.__indShow=_A;A.__np[0]=color_led_white;A.__np[3]=color_led_white;A.__np[5]=color_led_red;A.__np[6]=color_led_red
	def isTimeout(A,time):B=ticks_diff(time,A.__lastTime);return B>=A.__timeout_ms
	def update(A):
		B=ticks_ms()
		if A.isTimeout(B):A.changeIndicator(B)
	def changeIndicator(C):
		self.__lastTime=C;self.__indShow=not self.__indShow;A=color_led_off;B=color_led_off
		if self.__indShow:
			if direction!=Constants.LEFT:A=color_led_orange
			if direction!=Constants.RIGHT:B=color_led_orange
		self.__np[1]=A;self.__np[4]=A;self.__np[2]=B;self.__np[7]=B;self.__np.write()
class SpeedTicks:
	LIMIT=20
	def __init__(A):A.__index=-1;A.__times=[0]*A.LIMIT;A.__ticks=[0]*A.LIMIT;A.__countValues=-1;A.__lastTime=-1;A.isStopped=_B
	def getNewIndex(A,time):
		B=int(time/100000)
		if B==A.__lastTime:return-1
		else:A.__lastTime=B;return(A.__index+1)%A.LIMIT
	def isZeroChangeTicks(A,ticks):B=A.__ticks[A.__index]-ticks;return B==0
	def nextValues(A,newIndex,time,ticks):
		C=ticks;B=newIndex
		if A.__countValues<A.LIMIT:A.__countValues+=1
		if A.__countValues>2:A.isStopped=A.isZeroChangeTicks(C)
		A.__times[B]=time;A.__ticks[B]=C;A.__index=B
	def update(A,ticks):
		B=ticks_us();C=A.getNewIndex(B)
		if C>=0:A.nextValues(C,B,ticks)
	def calculate(B,count,offset):
		C=offset;A=count
		if A<2:A=10
		if A+C>=B.__countValues:A=B.__countValues-C-1
		if A<2:return 0
		D=B.__calculate(A,C);E=B.__calculate(A,C+1);return(D+E)/2
	def __calculate(A,count,offset):B=(A.__index-offset)%A.LIMIT;C=(B-count+1)%A.LIMIT;D=A.__times[B]-A.__times[C];E=A.__ticks[B]-A.__ticks[C];return 1000000*E/D
class Encoder:
	def __init__(A,place,ticksCount,radius):
		if place==Constants.LEFT:A.__pin=pin14
		else:A.__pin=pin15
		A.__ticksCount=ticksCount;A.__radius=radius;A.__speedTicks=SpeedTicks();A.__oldValue=A.readPin();A.ticks=0;A.direction=Constants.FORWARD
	def isStopped(A):return A.__speedTicks.isStopped
	def readPin(A):return A.__pin.read_digital()
	def nextTick(A,value):
		if A.direction==Constants.FORWARD:A.ticks+=1
		if A.direction==Constants.BACK:A.ticks-=1
	def update(A,direction):
		A.direction=direction;B=A.readPin()
		if B!=A.__oldValue:A.nextTick(B);A.__oldValue=B
		A.__speedTicks.update(A.ticks)
	def getSpeed(C,unit,count=2,offset=0):
		B=unit;A=C.__speedTicks.calculate(count,offset)
		if B==Constants.TicksPerSecond:return A
		A/=C.__ticksCount
		if B==Constants.CirclePerSecond:return A
		A*=TwoPI
		if B==Constants.RadianPerSecond:return A
		A*=C.__radius
		if B==Constants.MeterPerSecond:return A
		return 0
class Wheel:
	def __init__(A,place,radius,calibrateFactors):
		C=radius;B=place;A.__place=B;A.__encoder=Encoder(B,TICKS_PER_CIRCLE,C);A.__regulator=RegulatorP(6,125);A.__calibrateFactors=calibrateFactors;A.radius=C;A.speed=.0;A.direction=Constants.FORWARD;A.__pwmNoBack=0;A.__pwmNoForw=0
		if B==Constants.RIGHT:A.__pwmNoBack=2;A.__pwmNoForw=3
		elif B==Constants.LEFT:A.__pwmNoBack=4;A.__pwmNoForw=5
		i2c.write(MOTOR_I2C_ADDR,bytes([0,1]));i2c.write(MOTOR_I2C_ADDR,bytes([232,170]))
	def emergencyShutdown(A):A.speed=.0;A.writePWM(A.__pwmNoBack,A.__pwmNoForw,0)
	def isStopped(A):return A.__encoder.isStopped()
	def getMinimumSpeed(A):return A.__calibrateFactors.minSpeed*A.radius
	def writePWM(A,offPwmNo,onPwmNo,pwm):i2c.write(MOTOR_I2C_ADDR,bytes([offPwmNo,0]));i2c.write(MOTOR_I2C_ADDR,bytes([onPwmNo,pwm]));A.__pwm=pwm
	def getPwmFromSpeed(A,speed):
		B=speed
		if B==.0:return 0
		C=30;return A.__calibrateFactors.a*B+A.__calibrateFactors.b+C
	def calcAngularSpeedFromForwardSpeed(A,v):return v/A.radius
	def rideSpeed(A,v):
		A.speed=A.calcAngularSpeedFromForwardSpeed(v)
		if A.speed>=0:A.direction=Constants.FORWARD
		else:A.direction=Constants.BACK
		B=A.getPwmFromSpeed(abs(A.speed));A.__ridePwm(B)
	def checkMinimumPwm(A,pwm):
		B=pwm
		if A.speed!=.0:
			if A.isStopped():C=A.__calibrateFactors.minPwmWhenStopped
			else:C=A.__calibrateFactors.minPwmInMotion
			if B<C:B=C
		return B
	def __ridePwm(A,pwm):
		B=pwm;B=A.checkMinimumPwm(int(B))
		if B<0:return
		if B>255:return
		if A.__pwmNoForw>0 and A.__pwmNoBack>0:
			if A.direction==Constants.FORWARD:A.writePWM(A.__pwmNoBack,A.__pwmNoForw,B)
			if A.direction==Constants.BACK:A.writePWM(A.__pwmNoForw,A.__pwmNoBack,B)
	def __changePwm(B,changeValue):
		C=changeValue;A=0
		if B.direction==Constants.FORWARD:A=B.__pwm+C
		if B.direction==Constants.BACK:A=B.__pwm-C
		if A>255:A=255
		if A<0:A=0
		return B.__ridePwm(A)
	def getSpeed(A,unit,count=5,offset=0):return A.__encoder.getSpeed(unit,count,offset)
	def regulatePwm(A):
		B=ticks_ms()
		if A.__regulator.isTimeout(B):C=A.__encoder.getSpeed(Constants.RadianPerSecond);D=A.__regulator.getActionIntervention(B,A.speed,C);A.__changePwm(D)
	def update(A):A.__encoder.update(A.direction);A.regulatePwm()
class MotionControl:
	def __init__(A,wheelbase,wheelDiameter,velocity,calibrateLeft,calibrateRight):A.__d=wheelbase/2;A.__r=wheelDiameter/2;A.velocity=velocity;A.__wheelLeft=Wheel(Constants.LEFT,A.__r,calibrateLeft);A.__wheelRight=Wheel(Constants.RIGHT,A.__r,calibrateRight)
	def emergencyShutdown(A):
		try:A.newVelocity(0,0)
		except BaseException as B:A.__wheelLeft.emergencyShutdown();A.__wheelRight.emergencyShutdown();raise B
	def getMinimumSpeed(A):B=A.__wheelLeft.getMinimumSpeed();C=A.__wheelRight.getMinimumSpeed();return max(B,C)
	def newVelocityIfChanged(A,forward,angular):
		C=angular;B=forward
		if A.velocity.forward!=B or A.velocity.angular!=C:A.newVelocity(B,C)
	def newVelocity(A,forward,angular):C=angular;B=forward;A.velocity.forward=B;A.velocity.angular=C;A.__wheelLeft.rideSpeed(B-A.__d*C);A.__wheelRight.rideSpeed(B+A.__d*C)
	def update(A):A.__wheelLeft.update();A.__wheelRight.update()
class Robot:
	def __init__(A,leftCalibrate,rightCalibrate):B=Velocity();i2c.init(freq=400000);A.__senzors=Senzors();A.__regulatorDistance=RegulatorP(.5,500);A.__regulatorTurn=RegulatorP(.5,50);A.motionControl=MotionControl(.15,.067,B,leftCalibrate,rightCalibrate);A.motionControl.newVelocity(0,0);A.__controlTurnTime=ticks_ms();A.__turnLastAngular=0;A.__timeTurn=0;A.__timeTurnStop=0;A.displayText('X')
	def emergencyShutdown(A):A.motionControl.emergencyShutdown();A.displayText('S')
	def supplyVoltage(A):return .00898*pin2.read_analog()
	def displayText(A,text):0
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
	def controlSpeed(A):
		C=ticks_ms()
		if A.__regulatorDistance.isTimeout(C):
			D=.2;E=A.getObstacleDistance()
			if A.acceptableDistance(D,E):B=0
			else:B=A.__regulatorDistance.getActionIntervention(C,-D,-E);B=A.speedLimitation(B)
			A.motionControl.newVelocity(B,0)
	def getTurnCoef(B):A=B.__timeTurn-1;return A*A*.08+A*.45+1.
	def conditionChangeTurn(A,direction,back):
		D=direction;B=.1;C=0
		if D=='0':B=.05;C=A.__turnLastAngular
		else:
			A.__timeTurnStop=0
			if D=='L':B=.05;C=.01*A.getTurnCoef()
			if D=='R':A.__directionTurn=Constants.RIGHT;B=.05;C=-.01*A.getTurnCoef()
		A.__turnLastAngular=C
		if back:B*=-1
		A.motionControl.newVelocityIfChanged(B,C)
	def rideLine(A,back):
		B=back;C=ticks_ms()
		if A.__regulatorTurn.isTimeout(C):
			A.__controlTurnTime=C
			if A.__senzors.getSenzor(Senzors.LineTrackMiddle):A.conditionChangeTurn('C',B);A.__timeTurn=0
			elif A.__senzors.getSenzor(Senzors.LineTrackLeft):A.__timeTurn+=1;A.conditionChangeTurn('L',B)
			elif A.__senzors.getSenzor(Senzors.LineTrackRight):A.__timeTurn+=1;A.conditionChangeTurn('R',B)
			else:A.conditionChangeTurn('0',B)
	def getSituationLine(A):return A.__senzors.getSituationLine()
	def update(A):A.motionControl.update();A.__senzors.update();A.testBumber()
	def exitCrossRoads(B,direction):
		A=direction;C=0;D=.1
		if A==Constants.BACK:D*=-1
		if A==Constants.LEFT:C=.1
		if A==Constants.RIGHT:C=-.1
		B.motionControl.newVelocityIfChanged(D,C)
		if B.getSituationLine()==Constants.CrossRoads:B.__timeExitCRstart=ticks_ms();return _A
		if A==Constants.LEFT or A==Constants.RIGHT:
			if ticks_diff(ticks_ms(),B.__timeExitCRstart)<300:return _A
		return _B
	def turningToLine(A,direction,angularSpeed,timeTurnStart):
		C=angularSpeed;B=direction
		if B==Constants.LEFT:D=C
		if B==Constants.RIGHT:D=-C
		A.motionControl.newVelocityIfChanged(0,D)
		if ticks_diff(ticks_ms(),timeTurnStart)>1000:
			if A.__senzors.getSenzor(Senzors.LineTrackMiddle):return _B
		return _A
def localize_angular(command):
	A=command;global robotLocalizeAngular
	if A==Constants.LEFT:
		robotLocalizeAngular+=HalfPI
		if robotLocalizeAngular>PI:robotLocalizeAngular-=TwoPI
	if A==Constants.RIGHT:
		robotLocalizeAngular-=HalfPI
		if robotLocalizeAngular<=-PI:robotLocalizeAngular+=TwoPI
def localize_xy(command):
	global robotLocalizeAngular;global robotLocalizeX;global robotLocalizeY
	if command==Constants.BACK:robotLocalizeX-=int(cos(robotLocalizeAngular));robotLocalizeY-=int(sin(robotLocalizeAngular))
	else:robotLocalizeX+=int(cos(robotLocalizeAngular));robotLocalizeY+=int(sin(robotLocalizeAngular))
def print_localize(kde):global robotLocalizeAngular;global robotLocalizeX;global robotLocalizeY;display.clear();display.set_pixel(robotLocalizeY,robotLocalizeX,9)
def executeCommand(command):
	A=command;global timeTurnStart;global stateExecComm;global robot
	if stateExecComm==Constants.ST_Success or stateExecComm==Constants.ST_Failure:stateExecComm=Constants.ST_Start;robot.displayText('s')
	if stateExecComm==Constants.ST_Start:stateExecComm=Constants.ST_EC_ExitCrossRoads;robot.displayText('e')
	if stateExecComm==Constants.ST_EC_ExitCrossRoads:
		B=robot.exitCrossRoads(A)
		if B:
			if A==Constants.LEFT or A==Constants.RIGHT:stateExecComm=Constants.ST_EC_Turn;robot.displayText('t')
			else:stateExecComm=Constants.ST_Success;robot.displayText('o');return _B
	if stateExecComm==Constants.ST_EC_Turn:timeTurnStart=ticks_ms();stateExecComm=Constants.ST_EC_TurnToMiddleSenzor;robot.displayText('m')
	if stateExecComm==Constants.ST_EC_TurnToMiddleSenzor:
		B=robot.turningToLine(A,.41,timeTurnStart)
		if B:stateExecComm=Constants.ST_Success;robot.displayText('o');return _B
	return _A
def getActualCommand():
	global commands;global commandsIndex
	if commandsIndex<0:return Constants.FORWARD
	return commands[commandsIndex]
def nextCommand():global commands;global commandsIndex;commandsIndex+=1;return commandsIndex>=len(commands)
def stateMachine():
	global state;global robot;memory();D=_A
	if state==Constants.ST_Start:print_localize('1');robot.displayText('1');robot.displayText('L');state=Constants.ST_RidingLine
	if state==Constants.ST_RidingLine:
		robot.rideLine(getActualCommand()==Constants.BACK);A=robot.getSituationLine()
		if A==Constants.CrossRoads:state=Constants.ST_LocalizeBeforeExecCommand;robot.displayText('D')
		elif A==Constants.NONE:state=Constants.ST_Failure
	if state==Constants.ST_LocalizeBeforeExecCommand:
		localize_xy(getActualCommand());robot.displayText('2');print_localize('2');B=nextCommand()
		if B:state=Constants.ST_Success
		else:state=Constants.ST_ExecuteCommand
	if state==Constants.ST_ExecuteCommand:
		C=executeCommand(getActualCommand())
		if C:state=Constants.ST_LocalizeAfterExecCommand
	if state==Constants.ST_LocalizeAfterExecCommand:localize_angular(getActualCommand());robot.displayText('3');print_localize('3');state=Constants.ST_RidingLine
	if state==Constants.ST_Success:robot.displayText('S');robot.motionControl.newVelocity(0,0);return _B
	if state==Constants.ST_Failure:robot.displayText('F');robot.motionControl.newVelocity(0,0);return _B
	return _A
def memory():gc.collect()
if __name__=='__main__':
	print('Start');memory();robotLocalizeAngular=0;robotLocalizeX=0;robotLocalizeY=0;commands=[Constants.FORWARD,Constants.FORWARD,Constants.LEFT,Constants.LEFT,Constants.FORWARD,Constants.FORWARD,Constants.RIGHT,Constants.RIGHT,Constants.FORWARD,Constants.FORWARD];commandsIndex=-1;leftCalibrate=CalibrateFactors(1.,110,70,11.692,28.643);rightCalibrate=CalibrateFactors(1.,110,70,12.259,35.332);robot=None
	try:
		robot=Robot(leftCalibrate,rightCalibrate);memory();state=Constants.ST_Start;stateExecComm=Constants.ST_Start
		while not button_a.was_pressed():
			robot.update();isEnd=stateMachine()
			if isEnd:break
			sleep(1)
		robot.motionControl.newVelocity(0,0);print('Stop')
	except BaseException as e:
		if robot:robot.emergencyShutdown()
		print('Exception');raise e