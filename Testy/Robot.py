from microbit import i2c, pin19, pin20
from DirectionEnum import LEFT, RIGHT

class Robot:

    def __init__(self):

    def initialize(self):
        i2c.init(freq=400000, sda=pin20, scl=pin19)



def main():
    robot = Robot()
    robot.initialize()

    while not button_a.was_pressed():


        sleep(5)

