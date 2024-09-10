from microbit import pin2, button_a, sleep


class Robot:

    # .....

    def supplyVoltage(self):
        # Reading the ADC value
        adcValue = pin2.read_analog()
        # Convert ADC value to volts
        # Variables needed for conversion 3.3 V / 1024
        # (max. voltage at ADC pin / ADC resolution)
        voltaged = 0.00322265625 * adcValue
        # Multiply measured voltage by voltage divider ratio to calculate actual voltage
        # (10 kOhm + 5,6 kOhm) / 5,6 kOhm [(R1 + R2) / R2, Voltage divider ratio]
        return  voltaged * 2.7857142, 0.00897739927734375 * adcValue

    def supplyVoltageShort(self):
        return 0.00898 * pin2.read_analog()

if __name__ == "__main__":
    robot = Robot()
    while not button_a.get_presses():
        print(robot.supplyVoltage())
        print(robot.supplyVoltageShort())
        sleep(1000)

