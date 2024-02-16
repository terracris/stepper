import time
import ctypes
import numpy as np
from math import sqrt
import Jetson.GPIO as GPIO
from scipy.optimize import fsolve

class Stepper:
    CCW = 1
    CW = 0
    libc = ctypes.CDLL("libc.so.6") # Load the C library
    MILLISECONDS_IN_SECOND = 1000    # number of milliseconds in one second
    GPIO.setmode(GPIO.BOARD)

    def __init__(self, pulsePin, dirPin, enablePin):
        self.pulsePin = pulsePin
        self.dirPin = dirPin
        self.enablePin = enablePin
        self.direction = Stepper.CCW # current direction motor is spinning in

        self.currentPos = 0
        self.targetPos = 0
        # The current motor speed in steps per second --> Positive means counter-clockwise
        self.speed = 0
        self.maxSpeed = 0.0 # default to zero until set
        self.acceleration = 0.0 # default to zero until set
        self.stepInterval = 0 # the current interval between steps in microseconds (defines the angular velocity (RPM) and acceleration)
        self.minPulseWidth = 2.5 # the minimum pulse width in seconds (based on stepper driver)
        self.lastStepTime = 0 # the time the last step was done

        self.setOutputPins() # set up pins --> direction, pulse, enable

    """
    Set the target position and block until we get there.
    We use trapezoidal move profile. TODO use S curve. 

    absolute: the desired absolute position.

    """
    def moveAbsolute(self, absolute, interpolation_time):

        # if we are already where we want to be, don't do anything!        
        if self.currentPos == absolute:
            return
        else:
            self.targetPos = absolute
        
        # calculates the interpolation time in ms (resolution of all actions are in milliseconds)
        interpolation_time_in_ms = interpolation_time * Stepper.MILLISECONDS_IN_SECOND
        
        # get the number of pulses we have to travel
        delta_p = self.getChangeInPosition() # [ pulses ]
        
        # calculates the maximum velocity
        v_max = 1.5 * (delta_p / interpolation_time_in_ms)              # [ pulses / ms ]
        
        # time of each trapezoidal section in milliseconds.
        ta = tb = tc = interpolation_time_in_ms / 3                 # [ ms ]
        increment_time_resolution = 10 # [ ms ]

        a = (v_max / ta)          # [ p / ms^2 ]
        decelerating_distance = v_max * (1.675*ta)             # distance to start stopping
        
        pulses = 0
        decelerating = False

        pulse_start_time = self.getTime() # current time in milliseconds for pulses
        acceleration_start_time = pulse_start_time # current time in milliseconds for incrementing velocity by acceleration

        # we multiply acceleration increment time resolution since that is the velocity for the first 10 ms
        v_t = a * increment_time_resolution
        
        # while we are not where we want to be
        while self.currentPos != absolute:
            
            # get the time we have to wait until we can step
            self.stepInterval = 1 / v_t # this equation is given by the period. we want one pulse every v_t
            
            # while we havent reached our step interval, we wait
            # keep polling the time, once the current time - the start time >= our step interval we good
            if self.getTime() - pulse_start_time >= self.stepInterval:
                print(self.currentPos, v_t)
                # step interval has passed, so now we pulse
                self.step()
                # increment pulses travelled variable
                self.currentPos += 1
                pulses += 1
                # accelerates
                pulse_start_time = self.getTime()

            # check if it is time to decelerate
            if pulses >= decelerating_distance:
                decelerating = True

            # if we are accelerating, we increment v_t for the next step interval
            if not decelerating and self.getTime() - acceleration_start_time >= increment_time_resolution:
                v_t += a*increment_time_resolution
                acceleration_start_time = self.getTime()
                if v_t > v_max:
                    v_t = v_max
            
            if decelerating and self.getTime() - acceleration_start_time >= increment_time_resolution:
                v_t -= a * increment_time_resolution
                acceleration_start_time = self.getTime()
                # velocity should only be an integer
                # acceleration should also only be an integer (?) 
                if v_t <= 0:
                    v_t = 0.01
                
    
    def getChangeInPosition(self):
        return self.targetPos - self.currentPos
    
    def step(self):
        GPIO.output(self.pulsePin, GPIO.HIGH)
        Stepper.usleep(self.minPulseWidth) # TODO need to set minimum pulse width as 2.5 microseconds
        GPIO.output(self.pulsePin, GPIO.LOW)

    # returns time in microseconds since epoch 1970 
    def getTime(self):
        return int(time.time() * Stepper.MILLISECONDS_IN_SECOND)
    
    def setOutputPins(self):
        GPIO.setup(self.pulsePin, GPIO.OUT)   # output pin 
        GPIO.setup(self.dirPin, GPIO.OUT)     # output pin
        GPIO.setup(self.enablePin, GPIO.OUT)  # output pin

        GPIO.output(self.enablePin, GPIO.LOW) # turn motors on
        GPIO.output(self.dirPin, GPIO.HIGH)    # HIGH is ccw
        GPIO.output(self.pulsePin, GPIO.LOW)  # no pulse 

    @staticmethod
    def usleep(microseconds):
        """
        Sleep for the given number of microseconds.
        """ 
        Stepper.libc.usleep(int(microseconds))

    @staticmethod
    def calculate_acceleration(velocity, total_distance):

        def equation(delta, v_max, distance):
            # euler mascheroni constant
            euler_mascheroni_constant = 0.577
            return euler_mascheroni_constant * delta - v_max * np.exp(-distance * delta)
        
        # initial guess for delta
        initial_guess = 0.001

        # solve the equation
        delta_solution = fsolve(equation, initial_guess, args=(velocity, total_distance))[0]

        return round(delta_solution, 5)
   
if __name__ == '__main__':
    motor = Stepper(11,13,15)
    motor.moveAbsolute(1600, 1)
