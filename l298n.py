"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""

import time
import donkeycar as dk
import RPi.GPIO as gpio

class L298N:
    """
    PWM motor controler using PCA9685 and L298n boards.
    This is used for most RC Cars
    """
    def __init__(self, fwdLeftChannel, fwdRightChannel, bwdLeftChannel, bwdRightChannel, frequency=60):
        import Adafruit_PCA9685
        gpio.setmode(gpio.BCM)
        gpio.setwarnings(False)

        self.leftChannel = 0
        self.rightChannel = 1

        self.fwdRightChannel = 17
        self.bwdRightChannel = 4
        self.fwdLeftChannel = 27
        self.bwdLeftChannel = 22

        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(frequency)
        self.duty_cycle = 4095
        # self.fwdLeftChannel = fwdLeftChannel
        # self.fwdRightChannel = fwdRightChannel
        # self.bwdLeftChannel = bwdLeftChannel
        # self.bwdRightChannel = bwdRightChannel
        gpio.setup(self.fwdRightChannel, gpio.OUT)
        gpio.setup(self.bwdRightChannel, gpio.OUT)
        gpio.setup(self.fwdLeftChannel, gpio.OUT)
        gpio.setup(self.bwdLeftChannel, gpio.OUT)

        gpio.output(self.fwdRightChannel, False)
        gpio.output(self.bwdRightChannel, False)
        gpio.output(self.fwdLeftChannel, False)
        gpio.output(self.bwdLeftChannel, False)


    def set_pulse(self, pulse):
        # print(pulse)
        if pulse > 1:
            gpio.output(self.fwdRightChannel, True)
            gpio.output(self.bwdRightChannel, False)
            gpio.output(self.fwdLeftChannel, True)
            gpio.output(self.bwdLeftChannel, False)

            spd = int(self.duty_cycle*(pulse/100))
            self.pwm.set_pwm(self.leftChannel, 0, spd)
            self.pwm.set_pwm(self.rightChannel, 0, spd)
            # self.pwm.set_pwm(self.bwdLeftChannel, 0, 0)
            # self.pwm.set_pwm(self.bwdRightChannel, 0, 0)
        elif pulse < -1:
            gpio.output(self.fwdRightChannel, False)
            gpio.output(self.bwdRightChannel, True)
            gpio.output(self.fwdLeftChannel, False)
            gpio.output(self.bwdLeftChannel, True)

            spd = -int(self.duty_cycle*(pulse/100))
            self.pwm.set_pwm(self.leftChannel, 0, spd)
            self.pwm.set_pwm(self.rightChannel, 0, spd)
            # self.pwm.set_pwm(self.fwdLeftChannel, 0, 0)
            # self.pwm.set_pwm(self.fwdRightChannel, 0, 0)
            # self.pwm.set_pwm(self.bwdLeftChannel, 0, int(self.duty_cycle*(-pulse/100)))
            # self.pwm.set_pwm(self.bwdRightChannel, 0, int(self.duty_cycle*(-pulse/100)))
        else:
            gpio.output(self.fwdRightChannel, False)
            gpio.output(self.bwdRightChannel, False)
            gpio.output(self.fwdLeftChannel, False)
            gpio.output(self.bwdLeftChannel, False)

            # self.pwm.set_pwm(self.fwdLeftChannel, 0, 0)
            # self.pwm.set_pwm(self.fwdRightChannel, 0, 0)
            # self.pwm.set_pwm(self.bwdLeftChannel, 0, 0)
            # self.pwm.set_pwm(self.bwdRightChannel, 0, 0)

    def run(self, pulse):
        self.set_pulse(pulse)

    def shutdown(self):
        gpio.output(self.fwdRightChannel, False)
        gpio.output(self.bwdRightChannel, False)
        gpio.output(self.fwdLeftChannel, False)
        gpio.output(self.bwdLeftChannel, False)


class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self, controller=None,
                       max_pulse=100,
                       min_pulse=-100,
                       zero_pulse=0):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        #send zero pulse to calibrate ESC
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)


    def run(self, throttle):
        if throttle > 0:
            pulse = dk.util.data.map_range(throttle,
                                                    0, self.MAX_THROTTLE,
                                                    self.zero_pulse, self.max_pulse)
        else:
            pulse = dk.util.data.map_range(throttle,
                                                    self.MIN_THROTTLE, 0,
                                                    self.min_pulse, self.zero_pulse)
        self.controller.set_pulse(pulse)

    def shutdown(self):
        self.run(0) #stop vehicle
        gpio.cleanup()


class MockController(object):
    def __init__(self):
        pass

    def run(self, pulse):
        pass

    def shutdown(self):
        pass
