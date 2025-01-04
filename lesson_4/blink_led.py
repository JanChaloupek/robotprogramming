
# LEDky na robotu nejsou obycejne ledky,
# ale soucastka "NeoPixel"
# https://learn.adafruit.com/adafruit-neopixel-uberguide/the-magic-of-neopixels

from neopixel import NeoPixel
from microbit import pin0
from microbit import sleep

# vytvarime objekt typu NeoPixel, ktery se jmenuje napr "np"
# dokumentace https://microbit-micropython.readthedocs.io/en/v2-docs/neopixel.html
np = NeoPixel(pin0, 8)

# tento cyklus udela to, ze prvni ledka bude blikat s periodou 1s
# POZOR! Musite si zapnout napajeni z baterie

while True:
    # s objektem typu NeoPixel se pracuje stejne jako s polem
    # micropython rozlisuje mezi "list" a "array", coz jsou oba priklday polem
    # zakladni je "list" v micropythonu
    # https://docs.micropython.org/en/latest/genrst/builtin_types.html#list
    np[0] = (255, 255, 255)  # nastavim prvni ledku, tzn [0] na bilou (RGB hodnoty)
    # RGB = red, gree, blue - typicky format reprezentace barev v pocitaci
    # https://cs.wikipedia.org/wiki/RGB
    np.write()
    sleep(1000)
    np[0] = (0, 0, 0)  # nastavim prvni ledku, tzn [0] na cernou (RGB hodnoty)
    np.write()
    sleep(1000)

