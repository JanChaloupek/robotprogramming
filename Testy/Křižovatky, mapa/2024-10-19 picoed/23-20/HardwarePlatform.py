# Soubor pripravujici nazvy funkci jake pouziva microbit
from adafruit_ticks import ticks_diff as adf_ticks_diff, ticks_ms as adf_ticks_ms
from time import monotonic_ns, sleep as time_sleep
from board import P0, P8, P12, P14, P15, P19, P20
import digitalio
import displayio
from picoed import display, i2c as pico_i2c


pin0 = P0
pin8 = digitalio.DigitalInOut(P8)
pin12 = digitalio.DigitalInOut(P12)
pin14 = digitalio.DigitalInOut(P14)
pin15 = digitalio.DigitalInOut(P15)

_TICKS_PERIOD = const(1<<29)
_TICKS_MAX = const(_TICKS_PERIOD-1)
_TICKS_HALFPERIOD = const(_TICKS_PERIOD//2)

class I2C:
    def __init__(self):
        self.picoed_i2c = pico_i2c
        
    def init(self, freq=100_000, sda=P20, scl=P19):
        pass
    
    def __lock(self):
        while not self.picoed_i2c.try_lock():
            pass

    def __unlock(self):
        self.picoed_i2c.unlock()
        
    def scan(self):
        self.__lock()
        ret = self.picoed_i2c.scan()
        self.__unlock()
        return ret
       
    def read(self, addr, n, repeat=False):
        self.__lock()
        buffer = bytearray(n)
        self.picoed_i2c.readfrom_into(addr, buffer, start=0, end=n)
        self.__unlock()
        return buffer
    
    def write(self, addr, buf, repeat=False):
        self.__lock()
        self.picoed_i2c.writeto(addr, buf)
        self.__unlock()

i2c = I2C()

def pin_write_digital(pin, val):
    pin.direction = digitalio.Direction.OUTPUT    
    pin.value = (val!=1)

def pin_read_digital(pin):
    if pin.value:
        return 1
    return 0

def sleep(ms):
    time_sleep(ms / 1000)

def ticks_ms():
    return adf_ticks_ms()

def ticks_us():
    return monotonic_ns() // 1_000

def ticks_diff(ticks1, ticks2):
    return adf_ticks_diff(ticks1, ticks2)

def time_pulse_us(pin, pulse_level, timeout_us=1000000):
    start = ticks_us()
    while pin_read_digital(pin) != pulse_level:
        if ticks_diff(ticks_us(), start) > timeout_us:
            return -1  # Timeout
    start = ticks_us()
    while pin_read_digital(pin) == pulse_level:
        if ticks_diff(ticks_us(), start) > timeout_us:
            return -1  # Timeout
    return ticks_diff(ticks_us(), start)
