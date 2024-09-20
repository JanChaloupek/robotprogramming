from microbit import pin8, pin12, button_a, sleep
from machine import time_pulse_us

class Ultrazvuk:

    def __init__(self):
        self.__trigger = pin8
        self.__echo = pin12

        self.__trigger.write_digital(0)
        self.__echo.read_digital()

    def zmer_vzdalenost(self):

        rychlost_zvuku = 340 # m/s

        self.__trigger.write_digital(1)
        self.__trigger.write_digital(0)

        zmereny_cas_us = time_pulse_us(self.__echo, 1)
        if zmereny_cas_us < 0:
            return zmereny_cas_us

        zmereny_cas_s = zmereny_cas_us/ 1000000
        vzdalenost = zmereny_cas_s*rychlost_zvuku/2

        return vzdalenost


if __name__ == "__main__":

    ult = Ultrazvuk()

    while not button_a.was_pressed():
        aktualni_vzdalenost = ult.zmer_vzdalenost()
        if aktualni_vzdalenost < 0:
            # nejaky ten error :)
            print("ERROR")
        else:
            print(aktualni_vzdalenost)

        sleep(1000)

