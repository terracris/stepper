import time
import ctypes
import numpy as np
from math import sqrt
import Jetson.GPIO as GPIO
from collections import deque
from scipy.optimize import fsolve

class Stepper:
    CCW = 1
    CW = 0
    libc = ctypes.CDLL("libc.so.6") # Load the C library
    MILLISECONDS_IN_SECOND = 1000    # number of milliseconds in one second
    GPIO.setmode(GPIO.BOARD)

    """
    Each motor can have a defined positive direction for rotation
    """
    def __init__(self, pulsePin, dirPin, enablePin, stepsPerRev, gearRatio, positiveDirection, maxJointCCW, maxJointCW):
        self.pulsePin = pulsePin
        self.dirPin = dirPin
        self.enablePin = enablePin
        self.direction = Stepper.CCW if (positiveDirection == 1) else Stepper.CW # current direction motor is spinning ini
        self.positiveDirection = Stepper.CCW if (positiveDirection == 1) else Stepper.CW
        self.negativeDirection = Stepper.CW if (positiveDirection == 1) else Stepper.CCW


        self.currentPos = 0
        self.targetPos = 0
        # The current motor speed in steps per second --> Positive means counter-clockwise
        self.speed = 0
        self.maxSpeed = 0.0 # default to zero until set
        self.acceleration = 0.0 # default to zero until set
        self.stepInterval = 0 # the current interval between steps in microseconds (defines the angular velocity (RPM) and acceleration)
        self.minPulseWidth = 2.5 # the minimum pulse width in seconds (based on stepper driver)
        self.lastStepTime = 0 # the time the last step was done

        self.currentAngle = 0
        # gear ratio
        self.gear_ratio = gearRatio
        self.steps_per_rev = stepsPerRev
        self.step_angle = 360 / stepsPerRev
        self.maxJointLimitCCW = maxJointCCW
        self.maxJointLimitCW = maxJointCW
        self.setOutputPins() # set up pins --> direction, pulse, enable

   
    # write angles
    def write(self, angle):
        # check if the angle is within the limits for the joint
        # if it is, calculate the angles number of steps to get there
        
        if not self.inLimits(angle):
            print("angle out of limit")
            return
        
        # number of steps to go to from our current position
        steps = self.calculateSteps(angle)
        self.moveAbsolutePID(self.currentPos + steps)
        # technically, because I round the value, there is an error in my actual angle
        # actually, I should account for this error because it will build up immensely
        # TODO correct for the error
        self.currentAngle = self.updateAngle()

    
    def inLimits(self, angle):
        return angle >= self.maxJointLimitCW and angle <= self.maxJointLimitCCW
    
    """
    converts pulse position to angle in degrees.
    """
    def updateAngle(self):
        return (self.currentPos * self.step_angle) / self.gear_ratio

    
    """
    Converts desired angle to steps to control the stepper motor.
    angle: angle from -jointLimitCW to JointLimitCCW
    Changes the direction of rotation 
    """
    def calculateSteps(self, angle):
        current_angle = self.currentAngle
        change_in_angle = angle - current_angle

        # if the change in angle is negative, we move in that motor's negative direction
        if change_in_angle < 0:
            print("negative direction", self.negativeDirection)
            self.direction = self.negativeDirection
        else:
            self.direction = self.positiveDirection
        
        # rounds the number of steps required and then turns it into an int (just in case)
        goal_steps = int(round(change_in_angle * self.gear_ratio / self.step_angle))

        return goal_steps


    def moveAbsolutePID(self, absolute):
        # if we are not there set the target position
        if self.currentPos != absolute:
            self.targetPos = absolute
        else:
            return
        
        Kp = 0.005
        Ki = 0
        Kd = 0.003

        error_sum = 0  # Initialize error_sum outside the loop
        prev_error = 0  # Initialize prev_error outside the loop
        max_velocity = 10
        max_integral = 10
        start_time = self.getTime()

        while self.currentPos != absolute:
            error = self.getDistanceToTarget()
            error_sum += error
            error_sum = min(max_integral, error_sum)  # constrain integral
            error_der = error - prev_error
            prev_error = error

            v_t = abs(Kp * error + Ki * error_sum + Kd * error_der)  # constrain maximum velocity
            self.stepInterval = 1 / v_t  # [ms period between each pulse]

            if self.getTime() - start_time >= self.stepInterval:
                print(self.currentPos, v_t, self.direction)
                self.step()
                start_time = self.getTime()

    
    def getDistanceToTarget(self):
        return self.targetPos - self.currentPos
    
    def step(self):
        self.setDirectionPins()
        GPIO.output(self.pulsePin, GPIO.HIGH)
        Stepper.usleep(self.minPulseWidth) # TODO need to set minimum pulse width as 2.5 microseconds
        GPIO.output(self.pulsePin, GPIO.LOW)
        self.updatePosition()

    def updatePosition(self):
        if self.direction == self.positiveDirection:
            self.currentPos += 1
        else:
            self.currentPos -= 1
    
    def setDirectionPins(self):
        if self.direction == Stepper.CCW:
            print("ccw")
            GPIO.output(self.dirPin, GPIO.HIGH) # When direction pin is HIGH, the motor will spin CCW
        else:
            GPIO.output(self.dirPin, GPIO.LOW) # when direction pin is LOW, the motor will spin CW

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
   
if __name__ == '__main__':
    motor = Stepper(11,15,13, 200, 1, 0, 210, -10)
    motor.write(90)
