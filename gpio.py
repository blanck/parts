"""
actuators.py
Classes to control the motors and servos. These classes
are wrapped in a mixer class before being used in the drive loop.
"""

import time
import donkeycar as dk
import RPi.GPIO as GPIO

class GPIOCtrl:
    """
    PWM motor controler using PCA9685 and L298n boards.
    This is used for most RC Cars
    """
    def __init__(self):
        # Initialise the PCA9685 using the default address (0x40).
        GPIO.setwarnings(False)
        # GPIO.setmode(GPIO.BOARD)
        GPIO.setmode(GPIO.BCM)

        
        self.center = 0
        self.max = 0
        self.min = 0
        self.init_cal = 0

        self.error = 5
    # def set_angle(self, angle):
    #     steer_speed = int(abs(angle)*10)
    #     if angle > 0:
    #         print("left", steer_speed)
    #         GPIO.output(self.LB_BRK, GPIO.LOW)
    #         GPIO.output(self.RB_BRK, GPIO.LOW)
    #         GPIO.output(self.LB_FR, GPIO.LOW)
    #         GPIO.output(self.RB_FR, GPIO.LOW)
    #         self.pwmLeftSteering.start(steer_speed)
    #         self.pwmRightSteering.start(steer_speed)
    #     elif angle < 0:
    #         print("right", steer_speed)
    #         GPIO.output(self.LB_BRK, GPIO.LOW)
    #         GPIO.output(self.RB_BRK, GPIO.LOW)
    #         GPIO.output(self.LB_FR, GPIO.HIGH)
    #         GPIO.output(self.RB_FR, GPIO.HIGH)
    #         self.pwmLeftSteering.start(steer_speed)
    #         self.pwmRightSteering.start(steer_speed)
    #     else:
    #         self.pwmLeftSteering.start(0)
    #         self.pwmRightSteering.start(0)
    #         GPIO.output(self.LB_BRK, GPIO.HIGH)
    #         GPIO.output(self.RB_BRK, GPIO.HIGH)

    # def set_angle(self, angle, steering):
    #     if self.min == 0 or self.max == 0:
    #         self.calibrate(steering)
    #         return

    #     steer_left = self.center+((self.max-self.center)*angle)
    #     steer_right = self.center-((self.center-self.min)*angle)
    #     print(angle, steering, steer_left, self.center, steer_right)
    #     if angle < -0.1 and steering < steer_left and abs(steering - steer_left) > self.error:
    #         #left
    #         steer_speed = abs(steering - steer_left)**0.5*5
    #         if (steer_speed > 100): 
    #             steer_speed = 100
    #         GPIO.output(self.LB_FR, GPIO.HIGH)
    #         GPIO.output(self.RB_FR, GPIO.LOW)
    #         self.pwmLeftSteering.start(steer_speed)
    #         self.pwmRightSteering.start(0)
    #         # time.sleep(0.1)
    #         print("left", steer_speed)
    #     elif angle > 0.1 and steering < steer_right and abs(steering - steer_right) > self.error:
    #         #right
    #         steer_speed = abs(steering - steer_right)**0.5*5
    #         if (steer_speed > 100): 
    #             steer_speed = 100
    #         GPIO.output(self.LB_FR, GPIO.HIGH)
    #         GPIO.output(self.RB_FR, GPIO.LOW)
    #         self.pwmLeftSteering.start(0)
    #         self.pwmRightSteering.start(steer_speed)
    #         # time.sleep(0.1)
    #         print("right", steer_speed)
    #     elif steering < self.center-self.error*2 and abs(steering - self.center) > self.error:
    #         steer_speed = abs(self.center - steering)**.5*5
    #         if (steer_speed > 100): 
    #             steer_speed = 100
    #         self.pwmLeftSteering.start(steer_speed)
    #         self.pwmRightSteering.start(0)
    #         # time.sleep(0.1)
    #         print("left to center", steer_speed)
    #     elif steering > self.center+self.error*2 and abs(steering - self.center) > self.error:
    #         steer_speed = abs(self.center - steering)**.5*5
    #         if (steer_speed > 100): 
    #             steer_speed = 100
    #         self.pwmLeftSteering.start(0)
    #         self.pwmRightSteering.start(steer_speed)
    #         # time.sleep(0.1)
    #         print("right to center", steer_speed)
    #     else:
    #         self.pwmLeftSteering.start(0)
    #         self.pwmRightSteering.start(0)

    # def calibrate(self, steering):
    #     # self calibrate
    #     if steering == 0:
    #         print("Invalid steering ", steering)
    #         time.sleep(1)
    #     elif self.init_cal == 0:
    #         print("Calibrating wheels ")
    #         # full left
    #         self.pwmLeftSteering.start(50)
    #         time.sleep(2)
    #         self.pwmLeftSteering.start(0)
    #         self.init_cal = 1
    #     elif self.init_cal == 1:
    #         self.min = steering
    #         print("Calibrating min ", self.min)

    #         # full right
    #         self.pwmRightSteering.start(50)
    #         time.sleep(2)
    #         self.pwmRightSteering.start(0)
    #         self.init_cal = 2
    #     elif self.init_cal == 2:
    #         self.max = steering
    #         print("Calibrating max ", self.max)
    #         # calculate center
    #         self.center = self.min + (self.max - self.min)/2
    #         print("Calibrating center ", self.center)
    #         self.init_cal = 3

    # def set_pulse(self, pulse):
    #     # print("Set pulse ",pulse)
    #     if pulse > 0:
    #         #forward
    #         GPIO.output(self.LB_FR, GPIO.LOW)
    #         GPIO.output(self.RB_FR, GPIO.HIGH)
    #         self.pwmLeftSteering.start(pulse)
    #         self.pwmRightSteering.start(pulse)
    #         GPIO.output(self.LB_BRK, GPIO.LOW)
    #         GPIO.output(self.RB_BRK, GPIO.LOW)
    #     elif pulse < 0:
    #         # backward
    #         GPIO.output(self.LB_FR, GPIO.HIGH)
    #         GPIO.output(self.RB_FR, GPIO.LOW)
    #         self.pwmLeftSteering.start((-pulse))
    #         self.pwmRightSteering.start((-pulse))
    #         GPIO.output(self.LB_BRK, GPIO.LOW)
    #         GPIO.output(self.RB_BRK, GPIO.LOW)
    #     # else:
    #     #     GPIO.output(self.LB_BRK, GPIO.HIGH)
    #     #     GPIO.output(self.RB_BRK, GPIO.HIGH)
    #     #     self.pwmLeftSteering.start(0)
    #     #     self.pwmRightSteering.start(0)
            

    def run(self, pulse):
        self.set_pulse(pulse)
    def shutdown(self):
        GPIO.cleanup()

class PWMThrottle:
    """
    Wrapper over a PWM motor cotnroller to convert -1 to 1 throttle
    values to PWM pulses.
    """
    MIN_THROTTLE = -1
    MAX_THROTTLE =  1

    def __init__(self, controller=None,
                       max_pulse=50,
                       min_pulse=-50,
                       zero_pulse=0):

        self.controller = controller
        self.max_pulse = max_pulse
        self.min_pulse = min_pulse
        self.zero_pulse = zero_pulse

        #send zero pulse to calibrate ESC
        self.controller.set_pulse(self.zero_pulse)
        time.sleep(1)


    def run(self, throttle, angle):
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
        GPIO.output(self.LF_BRK, GPIO.HIGH)
        GPIO.output(self.LB_BRK, GPIO.HIGH)
        GPIO.output(self.RF_BRK, GPIO.HIGH)
        GPIO.output(self.RB_BRK, GPIO.HIGH)
        gpio.cleanup()

class GPIOMotor:
    """
    Motor controller
    """
    def __init__(self, motor_num):
        self.LF_BRK = 26
        self.LF_FR  = 21
        self.LF_SV  = 20

        self.LB_BRK = 12
        self.LB_FR  = 27
        self.LB_SV  = 18

        self.RF_BRK = 19
        self.RF_FR  = 16
        self.RF_SV  = 13

        self.RB_BRK = 6
        self.RB_FR  = 5
        self.RB_SV  = 24


        GPIO.setup(self.LB_BRK, GPIO.OUT)
        GPIO.setup(self.LB_FR, GPIO.OUT)
        GPIO.setup(self.LB_SV, GPIO.OUT)
        GPIO.setup(self.RB_BRK, GPIO.OUT)
        GPIO.setup(self.RB_FR, GPIO.OUT)
        GPIO.setup(self.RB_SV, GPIO.OUT)
        GPIO.setup(self.LF_BRK, GPIO.OUT)
        GPIO.setup(self.LF_FR, GPIO.OUT)
        GPIO.setup(self.LF_SV, GPIO.OUT)
        GPIO.setup(self.RF_BRK, GPIO.OUT)
        GPIO.setup(self.RF_FR, GPIO.OUT)
        GPIO.setup(self.RF_SV, GPIO.OUT)

        GPIO.output(self.LB_BRK, GPIO.LOW)
        GPIO.output(self.RB_BRK, GPIO.LOW)
        GPIO.output(self.LF_BRK, GPIO.LOW)
        GPIO.output(self.RF_BRK, GPIO.LOW)


        self.pwmLeftFrontSteering = GPIO.PWM(self.LF_SV, 120)
        self.pwmLeftBackSteering = GPIO.PWM(self.LB_SV, 120)
        self.pwmRightFrontSteering = GPIO.PWM(self.RF_SV, 120)
        self.pwmRightBackSteering = GPIO.PWM(self.RB_SV, 120)

        self.speed = 0
        self.throttle = 0
        self.motor_num = motor_num

    def map_range(self, x, X_min, X_max, Y_min, Y_max):
        '''
        Linear mapping between two ranges of values
        '''
        X_range = X_max - X_min
        Y_range = Y_max - Y_min
        XY_ratio = X_range/Y_range

        y = ((x-X_min) / XY_ratio + Y_min) // 1

        return int(y)


    def turn_off_motor(self):
        if self.motor_num == 0: #left
            GPIO.output(self.LF_BRK, GPIO.HIGH)
            GPIO.output(self.LB_BRK, GPIO.HIGH)
            self.pwmLeftFrontSteering.start(0)
            self.pwmLeftBackSteering.start(0)
        if self.motor_num == 1: #right
            GPIO.output(self.RB_BRK, GPIO.HIGH)
            GPIO.output(self.RF_BRK, GPIO.HIGH)
            self.pwmRightFrontSteering.start(0)
            self.pwmRightBackSteering.start(0)

    def turn(self, speed):
        '''
        Update the speed of the motor where 1 is full forward and
        -1 is full backwards.
        '''
        if speed > 1 or speed < -1:
            raise ValueError("Speed must be between 1(forward) and -1(reverse)")

        self.speed = speed
        self.throttle = int(self.map_range(abs(speed), -1, 1, -70, 70))

        if self.motor_num == 0: #left
            GPIO.output(self.LF_BRK, GPIO.LOW)
            GPIO.output(self.LF_FR, GPIO.LOW)
            GPIO.output(self.LB_BRK, GPIO.LOW)
            GPIO.output(self.LB_FR, GPIO.LOW)
            self.pwmLeftFrontSteering.start(self.throttle)
            self.pwmLeftBackSteering.start(self.throttle)
            if speed > 0:
                GPIO.output(self.LF_FR, GPIO.LOW)
                GPIO.output(self.LB_FR, GPIO.LOW)
            elif speed == 0:
                GPIO.output(self.LF_BRK, GPIO.HIGH)
                GPIO.output(self.LB_BRK, GPIO.HIGH)
            else:
                GPIO.output(self.LF_FR, GPIO.HIGH)
                GPIO.output(self.LB_FR, GPIO.HIGH)
        
        if self.motor_num == 1: #right
            GPIO.output(self.RF_BRK, GPIO.LOW)
            GPIO.output(self.RF_FR, GPIO.HIGH)
            GPIO.output(self.RB_BRK, GPIO.LOW)
            GPIO.output(self.RB_FR, GPIO.HIGH)
            self.pwmRightFrontSteering.start(self.throttle)
            self.pwmRightBackSteering.start(self.throttle)
            if speed > 0:
                GPIO.output(self.RF_FR, GPIO.HIGH)
                GPIO.output(self.RB_FR, GPIO.HIGH)
            elif speed == 0:
                GPIO.output(self.RF_BRK, GPIO.HIGH)
                GPIO.output(self.RB_BRK, GPIO.HIGH)
            else:
                GPIO.output(self.RF_FR, GPIO.LOW)
                GPIO.output(self.RB_FR, GPIO.LOW)




        # self.motor.setSpeed(self.throttle)



class PWMSteering:
    """
    Wrapper over a PWM motor cotnroller to convert angles to PWM pulses.
    """
    LEFT_ANGLE = -1
    RIGHT_ANGLE = 1

    def __init__(self, controller=None,
                       left_pulse=-50,
                       right_pulse=50):

        self.controller = controller
        self.angle = 0
        self.throttle = 0
        self.left_pulse = left_pulse
        self.right_pulse = right_pulse
        self.left_motor = GPIOMotor(0)
        self.right_motor = GPIOMotor(1)


    def run(self, throttle, angle):
        self.throttle = throttle
        self.angle = angle

        if throttle == 0 and angle == 0:
            self.stop()
        else:

            l_speed = ((self.left_motor.speed + throttle) / 3 - angle / 10)
            r_speed = ((self.right_motor.speed + throttle) / 3 + angle / 10)
            l_speed = min(max(l_speed, -1), 1)
            r_speed = min(max(r_speed, -1), 1)

            self.left_motor.turn(l_speed)
            self.right_motor.turn(r_speed)

        #map absolute angle to angle that vehicle can implement.
        # print(angle, steering)
        # self.controller.set_angle(angle)
        # pulse = dk.util.data.map_range(angle,
        #                                 self.LEFT_ANGLE, self.RIGHT_ANGLE,
        #                                 self.left_pulse, self.right_pulse)

        # self.controller.set_pulse(pulse)
    # def left(self, speed):
        

    # def right(self, speed):
    #     GPIO.output(self.LB_BRK, GPIO.LOW)
    #     GPIO.output(self.RB_BRK, GPIO.LOW)
    #     GPIO.output(self.LB_FR, GPIO.HIGH)
    #     GPIO.output(self.RB_FR, GPIO.HIGH)
    #     self.pwmLeftSteering.start(steer_speed)
    #     self.pwmRightSteering.start(steer_speed)

    def stop(self):
        self.left_motor.turn(0)
        self.right_motor.turn(0)
        

    def shutdown(self):
        self.run(0) #set steering straight


