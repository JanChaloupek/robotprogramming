_B=False
_A=.0
from neopixel import NeoPixel
from microbit import i2c,pin0,pin8,pin12,pin14,pin15,button_a,button_b,sleep
from utime import ticks_ms,ticks_us,ticks_diff
from machine import time_pulse_us
_Z=112
_AQ=40
class _J:_AC=0;_Q=1;_AI=2;_M=11;_C=12
class _O:_AE=0;_AH=1;_I=2
class _AT:_AR=1;_G=2;_AJ=3;_AA=4
class _AU:
	def __init__(A):A._EI=_A;A._DA=_A
class _F:
	def __init__(A,_FV,_FU,_FT,a,b):A._FS=_FV;A._FR=_FU;A._FQ=_FT;A.a=a;A.b=b
class _AK:
	def __init__(A,p,_ID):A._BO=p;A._CP=_ID;A._BQ=ticks_ms()
	def _FI(A,_HZ):B=ticks_diff(_HZ,A._BQ);return B>=A._CP
	def _EO(A,_HZ,_FC,_FB):A._BQ=_HZ;B=_FC-_FB;C=A._BO*B;return C
class _AN:
	_AG=64;_AF=32;_X=16;_W=8;_V=4
	def __init__(A):A._CP=50;A._GU()
	def _GU(A,_HZ=0):
		A._BE=i2c.read(56,1)[0]
		if _HZ==0:A._BT=ticks_ms()
		else:A._BT=_HZ
	def _EQ(A,_HH):return A._BE&_HH==0
	def _FI(A,_HZ):B=ticks_diff(_HZ,A._BT);return B>=A._CP
	def _IG(A):
		B=ticks_ms()
		if A._FI(B):A._GU(B)
class _P:
	_AC=0;_AM=1;_R=2
	def __init__(A):A._HB()
	def _HB(A):A._HI(A._AC)
	def _HI(A,_II):A._II=_II;A._HS=ticks_ms()
	def _FG(A,_GH):return A._II!=_GH
	def _IB(A):return ticks_diff(ticks_ms(),A._HS)>400
	def _DN(A):A._HI(A._AM if A._II==A._R else A._R)
	def _IG(A):
		if A._II!=A._AC:
			if A._IB():A._DN()
		else:A._HI(A._R)
class _T:
	_DS=0,0,0;_DT=100,35,0;_DW=60,60,60;_DX=255,255,255;_DU=60,0,0;_DV=255,0,0
	def __init__(A):A._CB=NeoPixel(pin0,8);A._CV=0
	def _HK(A,_FL,_DR):A._CB[_FL]=_DR
	def _HL(A,_FK,_DR):
		for B in _FK:A._HK(B,_DR)
	def _FI(A):return ticks_diff(ticks_ms(),A._CV)>100
	def write(A):A._CB.write();A._CV=ticks_ms()
class _U:
	_EX=1,2,4,7;_EY=1,4;_EZ=2,7;_ES=0,3;_DD=5,6;_FD=0,3,5,6;_HD=5,
	def __init__(A,_IJ):A._BU=_T();A._BJ=_P();A._CS=_IJ;A._HN(_O._AH);A._HM();A._HO();A._BM=_B;A._AY=0;A._IK=_B
	def _HN(A,_II):A._BW=_II
	def _HM(A):
		if A._CS._DA>_A:A._BI=_J._Q
		elif A._CS._DA<_A:A._BI=_J._AI
		else:A._BI=_J._AC
	def _HO(A):A._BN=A._CS._EI<_A
	def _HJ(A):
		if _B:A._BM=True;A._AY=ticks_ms()
	def _FF(A):
		if A._BM:
			B=ticks_diff(ticks_ms(),A._AY)
			if B<1000:return True
			A._BM=_B
		return _B
	def _IG(A):
		A._HM();A._HO();A._HJ();D=A._BJ._II
		if A._BI!=_J._AC or A._IK:A._BJ._IG()
		else:A._BJ._HB()
		if A._BJ._FG(D)or A._BU._FI():
			if A._BJ._II==_P._R:
				if A._BF==_J._Q or A._IK:A._BU._HL(A._EY,_T._DT)
				if A._BF==_J._AI or A._IK:A._BU._HL(A._EZ,_T._DT)
			else:A._BU._HL(A._EX,_T._DS)
			C=_T._DS;B=_T._DS
			if A._BW==_O._AH:C=_T._DW;B=_T._DU
			if A._BW==_O._I:C=_T._DX;B=_T._DU
			if A._FF():B=_T._DV
			A._BU._HL(A._ES,C);A._BU._HL(A._DD,B)
			if A._BN:A._BU._HL(A._HD,_T._DW)
			A._BU.write()
class _AP:
	_S=50
	def __init__(A):A._BK=-1;A._CQ=[0]*A._S;A._CO=[0]*A._S;A._BC=-1;A._BS=-1;A._FH=True
	def _EM(A,_HZ):
		B=int(_HZ/100000)
		if B==A._BS:return-1
		else:A._BS=B;return(A._BK+1)%A._S
	def _GB(A,_FY,_HZ,_HV):
		C=_HV;B=_FY
		if A._BC<A._S:
			A._BC+=1
			if A._BC>2:A._FH=A._CO[A._BK]-C==0
		A._CQ[B]=_HZ;A._CO[B]=C;A._BK=B
	def _IG(A,_HV):
		B=ticks_us();C=A._EM(B)
		if C>=0:A._GB(C,B,_HV)
	def _DI(B,_DY=5,_GE=0):
		C=_GE;A=_DY
		if A<2:A=10
		if A+C>=B._BC:A=B._BC-C-1
		if A<2:return 0
		D=B._AZ(A,C);E=B._AZ(A,C+1);return(D+E)/2
	def _AZ(A,_DY,_GE):B=(A._BK-_GE)%A._S;C=(B-_DY+1)%A._S;D=A._CQ[B]-A._CQ[C];E=A._CO[B]-A._CO[C];return 1000000*E/D
class _L:
	def __init__(A,_GP):
		if _GP==_J._Q:A._CD=pin14
		else:A._CD=pin15
		A._CN=_AP();A._CC=A._GV();A._HV=0;A._EA=_J._M
	def _FH(A):return A._CN._FH
	def _GV(A):return A._CD.read_digital()
	def _GA(A):
		if A._EA==_J._M:A._HV+=1;return 0
		if A._EA==_J._C:A._HV-=1;return 0
		return-1
	def _IG(A,_EA):
		A._EA=_EA;B=A._GV()
		if B!=A._CC:A._GA();A._CC=B
		A._CN._IG(A._HV)
	def _ER(C,_IF,_DY=5,_GE=0):
		B=_IF;A=C._CN._DI(_DY,_GE)
		if B==_AT._AR:return A
		A/=_AQ
		if B==_AT._G:return A
		A*=6.2832
		if B==_AT._AJ:return A
		return 0
class _AV:
	def __init__(A,_GP,_GR,_DK):
		B=_GP;A._CE=B;A._BH=_L(B);A._CJ=_AK(6,500);A._BA=_DK;A._GR=_GR;A._HQ=_A;A._EA=_J._M
		if B==_J._AI:A._CG=2;A._CH=3
		elif B==_J._Q:A._CG=4;A._CH=5
		else:A._CG=0;A._CH=0
		i2c.write(_Z,bytes([0,1]));i2c.write(_Z,bytes([232,170]))
	def _EF(A):A._HQ=_A;A._IQ(A._CG,A._CH,0)
	def _FH(A):return A._BH._FH()
	def _EL(A):return A._BA._FS
	def _IQ(A,_GD,_GF,_GQ):i2c.write(_Z,bytes([_GD,0]));i2c.write(_Z,bytes([_GF,_GQ]));A._CF=_GQ
	def _EP(A,_HQ):
		B=_HQ
		if B==_A:return 0
		return A._BA.a*B+A._BA.b
	def _HE(A,_HQ):
		B=_HQ;A._HQ=B
		if A._HQ>=0:A._EA=_J._M
		else:A._EA=_J._C
		C=A._EP(abs(B));A._CK(C)
	def _DP(A,_GQ):
		B=_GQ
		if A._HQ!=_A:
			if A._FH():C=A._BA._FR
			else:C=A._BA._FQ
			if B<C:B=C
		return B
	def _CK(A,_GQ):
		B=_GQ;C=B;B=int(B);B=A._DP(B)
		if B<0:return
		if B>255:return
		if A._CH>0 and A._CG>0:
			if A._EA==_J._M:return A._IQ(A._CG,A._CH,B)
			if A._EA==_J._C:return A._IQ(A._CH,A._CG,B)
			return
	def _BB(B,_DO):
		C=_DO;A=0
		if B._EA==_J._M:A=B._CF+C
		if B._EA==_J._C:A=B._CF-C
		if A>255:A=255
		if A<0:A=0
		return B._CK(A)
	def _ER(A,_IF,_DY=5,_GE=0):
		C=_GE;B=_DY
		if _IF==_AT._AA:return A._GR*A._BH._ER(_AT._AJ,B,C)
		return A._BH._ER(_IF,B,C)
	def _GY(A):
		B=ticks_ms()
		if A._CJ._FI(B):C=A._BH._ER(_AT._AJ);D=A._CJ._EO(B,A._HQ,C);A._BB(D)
	def _IG(A):A._BH._IG(A._EA);A._GY()
class _AB:
	def __init__(A,_IN,_IM,_IJ,_DL,_DM):A._BD=_IN/2;A._CI=_IM/2;A._IJ=_IJ;A._CT=_AV(_J._Q,A._CI,_DL);A._CU=_AV(_J._AI,A._CI,_DM)
	def _EF(A):
		try:A._FZ(0,0)
		except _D as B:A._CT._EF();A._CU._EF();raise B
	def _EL(A):B=A._CT._EL();C=A._CU._EL();return max(B,C)
	def _FZ(A,_EI,_DA):A._IJ._EI=_EI;A._IJ._DA=_DA;A._CT._HE(A._IJ._EI-A._BD*A._IJ._DA);A._CU._HE(A._IJ._EI+A._BD*A._IJ._DA)
	def _DH(A,_HQ):return _HQ/A._CI
	def _IG(A):A._CT._IG();A._CU._IG()
class _AO:
	_Y=10
	def __init__(A,_IC):A._CR=pin8;A._CR.write_digital(0);A._BG=pin12;A._BG.read_digital();A._BP=0;A._BR=-3;A._CP=_IC;A._FJ=-1
	def _DJ(A,_HZ):
		A._BP=_HZ;A._CR.write_digital(1);A._CR.write_digital(0);C=340;B=time_pulse_us(A._BG,1,5000)
		if B<0:return B
		D=B/1000000;E=D*C/2;return E
	def _FI(A,_HZ):B=ticks_diff(_HZ,A._BP);return B>=A._CP
	def _IG(A):
		B=ticks_ms()
		if A._FI(B):
			A._BR=A._DJ(B)
			if A._BR>0:A._FJ=A._BR
			if A._BR==-1:A._FJ=A._Y
class _AL:
	def __init__(A,_FM,_HF):B=_AU();i2c.init(freq=400000);A._CL=_AN();A._CM=_AO(300);A._CJ=_AK(15,1000);A._BV=_U(B);A._FW=_AB(.15,.067,B,_FM,_HF);A._FW._FZ(0,0);A._BY=A._FW._DH(.3);A._BZ=A._FW._EL()
	def _EF(A):A._FW._EF()
	def _HT(A):return .00898*_GN.read_analog()
	def _EN(A):return A._CM._FJ
	def _HU(A):
		if A._CL._EQ(_AN._AF)or A._CL._EQ(_AN._AG):A._FW._FZ(0,0)
	def _HR(B,_HQ):
		C=_HQ
		if C>=0:D=1
		else:D=-1
		A=abs(C)
		if A>B._BY:A=B._BY
		elif A<B._BZ:A=B._BZ
		return D*A
	def _CY(B,_HA,_EB):A=abs(_EB-_HA);return A<=.03
	def _GZ(A):
		C=ticks_ms()
		if A._CJ._FI(C):
			D=.2;E=A._EN()
			if A._CY(D,E):B=0
			else:B=A._CJ._EO(C,-D,-E);B=A._HR(B)
			A._FW._FZ(B,0)
	def _IG(A):A._FW._IG();A._CL._IG();A._CM._IG();A._BV._IG();A._HU();A._GZ()
if __name__=='__main__':
	_FM=_F(2.8,110,75,11.692,28.643);_HF=_F(2.8,110,75,12.259,30.332);_HG=_AL(_FM,_HF)
	try:
		while not button_a.was_pressed():_HG._IG();sleep(1)
		_HG._FW._FZ(0,0)
	except _D as e:_HG._EF();raise e
