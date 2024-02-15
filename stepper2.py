import time
from math import sqrt
import Jetson.GPIO as GPIO
import ctypes

class Stepper:
    CCW = 1
    CW = 0
    libc = ctypes.CDLL("libc.so.6") # Load the C library
    MICROSECONDS_IN_SECOND = 1e6    # number of microseconds in one second
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
        
        # get the number of pulses we have to travel
        delta_p = self.getChangeInPosition()
        # calculates the maximum velocity
        v_max = delta_p / interpolation_time
        ta = tb = tc = interpolation_time / 3
        a = v_max / ta
        decelerating_distance = v_max * (1.5*ta) # distance to start stopping
        pulses = 0
        decelerating = False

        
        start_time = int(time.time() * Stepper.MICROSECONDS_IN_SECOND) # current time in microseconds since the epoch
        # while we are not where we want to be
        while self.currentPos != absolute:
            # calculate the change in position
            current_time = int(time.time() * Stepper.MICROSECONDS_IN_SECOND) # current time in microseconds
            change_in_time = current_time - start_time

            # accelerates
            if not decelerating:
                v_t += a*change_in_time
                if v_t > v_max:
                    v_t = v_max

            if pulses == decelerating_distance:
                decelerating = True
            
            if decelerating:
                v_t -= a*change_in_time
                # velocity should only be an integer
                # acceleration should also only be an integer (?) 
                if v_t <= 0:
                    return

            self.stepInterval = Stepper.MICROSECONDS_IN_SECOND / v_t

            if change_in_time >= self.stepInterval:
                pulses += 1
                self.step()
                start_time = int(time.time() * Stepper.MICROSECONDS_IN_SECOND)
    
    def getChangeInPosition(self):
        return self.targetPos - self.currentPos
    
    def step(self):
        print("stepping")
        GPIO.output(self.pulsePin, GPIO.HIGH)
        Stepper.usleep(self._minPulseWidth) # TODO need to set minimum pulse width as 2.5 microseconds
        GPIO.output(self.pulsePin, GPIO.LOW)

    # returns time in microseconds since epoch 1970 
    def getTime(self):
        return int(time.time() * 1e6)