from HardwarePlatform import P1, P13, sleep
import pwmio

# Inicializace PWM pro piny
pwm1 = pwmio.PWMOut(P1, frequency=100)

# Funkce pro přepočet hodnoty (0-180°) na šířku pulzu (1000 - 2000 mikrosekund)
def scale(num, in_min, in_max, out_min, out_max):
    return int((num - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

# Funkce pro ovládání serva
def servo(pwm, value):
    print(value)
    pwm.duty_cycle = value

while True:
    servo(pwm1, 3000)  # Kanál 1 - 0° (vpravo)
    sleep(1000)  # Pauza 2 sekundy
    servo(pwm1, 16000)  # Kanál 1 - 180° (vlevo)
    sleep(3000)  # Pauza 2 sekundy
