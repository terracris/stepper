import time
from math import sqrt
import Jetson.GPIO as GPIO
import ctypes

# we are using stepper motor drivers and we only care about two pins
class Stepper:
    CCW = 1
    CW = 0
    libc = ctypes.CDLL("libc.so.6") # Load the C library
    GPIO.setmode(GPIO.BOARD)


    def __init__(self, pulsePin, dirPin, enablePin):
        self.pulsePin = pulsePin
        self.dirPin = dirPin
        self.enablePin = enablePin
        self.direction = Stepper.CCW # current direction motor is spinning in

        self._currentPos = 0
        self._targetPos = 0
        # The current motor speed in steps per second --> Positive is clockwise (steps per second)
        self._speed = 0.0
        self._maxSpeed = 0.0 # default to zero until set
        self._acceleration = 0.0 # default to zero until set
        self._sqrt_twoa = 1.0
        self._stepInterval = 0 # the current interval between steps in microseconds. 0 measns the motor is stopped
        self._minPulseWidth = 2.5 # time in microseconds
        self._lastStepTime = 0 # The time the last step was done

        self._n = 0 # the step counter for speed calculations
        self._c0 = 0.0 # initial step size in microseconds
        self._cn = 0.0 # last step size in microseconds
        self._cmin = 1.0 # min step size in microseconds based on maxSpeed

        # set up pins --> direction, pulse, enable and sets them LOW
        self.setOutputPins()
        
        # some reasonable defaults
        self.setAcceleration(1)
        self.setMaxSpeed(1)


    """
    Set the target position. The run() function will try to move the motor (at most one step per call)
    from the current position to the target position set by the most recent call to the moveTo function

    moveTo also recalculates the speef for the next step.
    If we want to use contant speed movements, you should call setSpeed() after calling moveTo().

    absolute: the desired absolute position. Negative is counterclockwise from the 0 position. (start up position)
    """
    def moveAbsolute(self, absolute):
        if self._targetPos != absolute:
            self._targetPos = absolute
            self.computeNewSpeed() # get the new speed to move to a position


    """
    Set the target position relative to the current position.

    relative: the desired position relative to the current position. Negative is counterclockwise from the current position.
    """
    def moveRelative(self, relative):
        self.moveAbsolute(self._currentPos + relative)

    """
    Poll the motor and step it if a step is due, implementing accelerations and decelerations to achieve the target
    position. You must call this as frequently as possible, but at least once per minimum step time interval,
    preferably in your main loop. 

    Note that each call to run() will make at most one step, and only when a step is due, based on the current speed
    and the time since the last step.

    Returns a boolean. True if the motor is still running to the target position.
    """
    def run(self):
        if(self.runSpeed()):
            self.computeNewSpeed()
        
        return self._speed != 0.0 or self.getDistanceToGo != 0

    """
    Poll the motor and step it if a step is due, implementing a constant speed as set by the most recent
    call to setSpeed(). You must call this as frequently as possible but at least once per step interval,

    Returns a boolean. True if the motor was stepped.
    """
    def runSpeed(self):
        # don't do anything unless we actually have a step interval 
        if not self._stepInterval:
            return False
        
        # number of microseconds
        currTime = time.time() * 1e6

        if currTime - self._lastStepTime >= self._stepInterval:
            if self.direction == Stepper.CW:
                # clockwise
                self._currentPos += 1
            else:
                # counter clockwise
                self._currentPos -= 1

            self.step(self._currentPos)
            self._lastStepTime = currTime # caution: does not account for costs in step()

            return True
        
        return False
            

    """
    Sets the maximum permitted speed. The run() function will accelerate up to the speed set by this function.

    Caution: the maximum speed achieveable depedns on your processor and clock speed.
             the default is 1.0 steps per second.

    float speed: the desired maximum speed in steps per second. Must be > 0. 

    Caution: Speeds that exceed the maximum speed supported by the processor may result in non-linear accelerations
             and decelerations. 
    """
    def setMaxSpeed(self, speed):
        if speed < 0.0:
            speed = -speed
        if self._maxSpeed != speed:
            self._maxSpeed = speed
            self._cmin = 1e6 / speed
        
        # recompute _n from current speed and adjust speed if acceleratinr or cruising
        if self._n > 0:
            self._n = pow(speed, 2) / (2.0 * self._acceleration)
            self.computeNewSpeed()

    """
    Returns the maximum speed configured for this stepper that was previously set by setMaxSpeed()
    """
    def getMaxSpeed(self):
        self._maxSpeed

    """
    Sets the acceleration/deceleration rate. 
    
    float acceleration: the desired acceleration in steps per second per second. Must be > 0.

    This is an expensive call since it requires a square root to be calculated. Don't call more often than needed.
    """
    def setAcceleration(self, acceleration):
        if acceleration == 0:
            return
        if acceleration < 0.0:
            acceleration = -acceleration
        
        if self._acceleration != acceleration:
            # recompute _n per equation 17 TODO check documentation for equation
            self._n = self._n * (self._acceleration / acceleration)

            # new c0 per Equation 7, with correction per Equation 15
            self._c0 = 0.676 * sqrt(2.0 / acceleration) * 1e6
            self._acceleration = acceleration
            self.computeNewSpeed()

    """
    Returns the acceleration/deceleration rate configured for this stepper previously set by setAcceleration()
    """
    def getAcceleration(self):
        self._acceleration
    
    """
    Sets the desired constant speed for use with runSpeed()

    float speed: The desired constance speed in steps per second.
                 Positive is clockwise. Speeds of more than 1000 steps per second are unreliable.
                 Very slow speeds may be set (eg 0.00027777 for once per hour)

                 Speed accuracy depends on clock speed.
                 The speed will be limited by the current value of setMaxSpeed()
    """
    def setSpeed(self, speed):
        if self._speed == speed:
            return
        
        speed = Stepper.constrain(speed, -self._maxSpeed, self._maxSpeed)
        if(speed == 0.0):
            self._stepInterval = 0
        else:
            self._stepInterval = abs(1e6 / speed)
            self.direction = Stepper.CW if (speed > 0.0) else Stepper.CCW

        self._speed = speed

    """
    Returns the most recent speed in steps per second.
    """
    def getSpeed(self):
        self._speed

    """
    Returns the distance from the current position to the target position in steps.
    Positive is clockwise from the current position.
    """
    def getDistanceToGo(self):
        return self._targetPos - self._currentPos

    """
    Returns the most recently set target position.
    """
    def getTargetPosition(self):
        return self._targetPos

    """
    Returns the current motor position in steps. Positive is clockwise from the 0 position.
    """
    def getCurrentPosition(self):
        return self._currentPos

    """
    Resets the current position of the motor, so that wherever the motor happens to be right now is considered
    to be the new zero position on a stepper after an initial hardware positioning move.

    Has the side effect of setting the current motor speed to 0.

    position: the position in steps of wherever the motor happens to be right now.

    Useful during initialisations or after initial positioning
    Sets speed to 0
    """
    def setCurrentPosition(self, position):
        self._targetPos = position
        self._currentPos = position

        self._n = 0
        self._stepInterval = 0
        self._speed = 0.0

    """
    Moves the motor (with acceleration/deceleration) to the target position and blocks
    until it is at position. Dont use this in event loops since it blocks.

    Don't worry about this function being blocking. We are going to use threads.
    """
    def runToPosition(self):

        while(self.run()):
            pass
        

    """
    Executes runSpeed() unless the targetPosition is reached.
    This function needs to be called often just like runSpeed() or run()
    Will steo the motor if a step is required at the currently selected speed unless the target
    position has been reached. Does not implement accelerations.
    """
    def runSpeedToPosition(self):
        if self._targetPos == self._currentPos:
            return False
        
        if self._targetPos > self._currentPos:
            self.direction = Stepper.CW
        else:
            self.direction = Stepper.CCW
        
        return self.runSpeed()

    """
    Moves the motor (with acceleration/deceleration) to the new target position. Dont use this in event
    loops, since it blocks.
    """
    def runToNewPosition(self, position):
        self.moveTo(position)
        self.runToPosition()


    """
    Sets a new target position that causes the stepper to stop as quickly as possible, using the current speed
    and acceleration parameters.
    """
    def stop(self):
        if self._speed != 0.0:
            stepsToStop = (pow(self._speed, 2) / (2.0 * self._acceleration)) + 1 # TODO Review Equation 16
            if self._speed > 0:
                self.moveRelative(stepsToStop)
            else:
                self.moveRelative(-stepsToStop)


    """
    Sets the minimum pulse width allowed by the stepper driver. The minimum practical pulse width is 
    approximately 20 microseconds. Times less than 20 microseconds will usually result in 20 microseconds or so.

    Our driver says minimum is can be up to 5
    """
    def setMinPulseWidth(self, minWidth):
        self._minPulseWidth = minWidth

    """
    Returns true if the motor is speed is not zero or nto at the target position.
    """
    def isRunning(self):
        return not (self._speed == 0.0 and (self._targetPos == self._currentPos))

    """
    Computes a new instantaneous speed and set that as the current speed.
    It is called :
        after each step
        after change to maxSpeed through setMaxSpeed()
        after change to acceleration through setAcceleration()
        after change to target position (relative or absolute) through moveAbsolute() or moveAbsolute

        return the new step interval
    """
    def computeNewSpeed(self):
        distanceTo = self.getDistanceToGo()
        stepsToStop = pow(self._speed, 2) / (2.0 * self._acceleration) # TODO review Equation 16

        if distanceTo == 0 and stepsToStop <= 1:
            self._stepInterval= 0
            self._speed = 0.0
            self._n = 0

            return self._stepInterval
        
        # we are counterclockwise from the target
        # need to go clockwise from here, maybe decelerate now
        if distanceTo > 0:
            # currently accelerating, need to decel now? Or maybe going the wrong way?
            if self._n > 0:
                if stepsToStop >= distanceTo or self.direction == Stepper.CCW:
                    self._n = -stepsToStop # start deceleration now

            # currently decelerating, need to accel again?
            elif self._n < 0:
                if stepsToStop < distanceTo and self.direction == Stepper.CW:
                    self._n = -self._n

        elif distanceTo < 0:
            # we are clockwise from the target.
            # need to go counterclockwise from here, maybe decelerate

            if self._n > 0:
                # currently accelerating, need to decel now? Or maybe going the wrong way?
                if (stepsToStop >= -distanceTo) or self.direction == Stepper.CW:
                    self._n = -stepsToStop # start deceleration
            elif self.n < 0:
                if (stepsToStop < -distanceTo) and self.direction == Stepper.CCW:
                    self._n = -self._n # start accelerating

        # need to accelerate
        if (self._n == 0):
            # first step from stopped
            self._cn = self._c0
            self.direction = Stepper.CW if (distanceTo > 0) else Stepper.CCW
        else:
            # subsequent step. works for accel
            self._cn = self._cn - ((2.0 * self._cn) / ((4.0 * self._n) + 1)) # TODO review equation 13
            self._cn = max(self._cn, self._cmin)
        
        self._n += 1
        self._stepInterval = self._cn
        self._speed = 1e6 / self._cn
        if self.direction == Stepper.CCW:
            self._speed = -self._speed

        return self._stepInterval
            

    """
    Called to execute a step. Only called when a step is required.
    The minimum pulse width for the Stepperonline DM542T is 2.5 microseconds.
    """
    def step(self):
        
        #TODO in the AccelStepper library they set the direction pins each time?
        # should I also do that? how would I change direction?
        # not enough familiarity to know how this will effect code performance
        print("stepping")
        GPIO.output(self.pulsePin, GPIO.HIGH)
        Stepper.usleep(self._minPulseWidth) # TODO need to set minimum pulse width as 2.5 microseconds
        GPIO.output(self.pulsePin, GPIO.LOW)

    """
    Sets the direction pin for the stepper to rotate in. 
    """
    def setDirection(self, direction):
        # TODO verify that these directions are correct
        rotation = GPIO.HIGH if (direction == Stepper.CCW) else GPIO.LOW 
        GPIO.output(self.dirPin, rotation)

    """
    Gives power to stepper motor
    """
    def enableStepper(self):
        GPIO.output(self.enablePin, GPIO.LOW)

    """
    Removes power to stepper motor
    """
    def disableStepper(self):
        GPIO.output(self.enablePin, GPIO.HIGH)

    def setOutputPins(self):
        GPIO.setup(self.pulsePin, GPIO.OUT)   # output pin 
        GPIO.setup(self.dirPin, GPIO.OUT)     # output pin
        GPIO.setup(self.enablePin, GPIO.OUT)  # output pin

        GPIO.output(self.enablePin, GPIO.LOW) # turn motors on
        GPIO.output(self.dirPin, GPIO.LOW)    # I think low is clockwise
        GPIO.output(self.pulsePin, GPIO.LOW)  # no pulse 

    """
    Constrains a value between a minimum and maximum value.
    """
    @staticmethod
    def constrain(value, min_value, max_value):
        return min(max_value, max(min_value, value))
    
    @staticmethod
    def usleep(microseconds):
        """
        Sleep for the given number of microseconds.
        """ 
        Stepper.libc.usleep(int(microseconds))

if __name__ == '__main__':
    motor = Stepper(11, 13, 15)
    motor.setMaxSpeed(4000) # max speed is 4000 pulses per second
    motor.setSpeed(0)
    motor.setAcceleration(1000)
    time.sleep(1)
    # TODO make set speed blocking?
    motor.setSpeed(1000) # sets speed to 1000 pulses per second
    # will run the motor at the defined speed until count is achieved.
    while True:    
        # GPIO.output(motor.pulsePin, GPIO.HIGH)
        motor.step()
    # TODO make moving to a position blocking. idk if it makes sense to make speed blocking?
    # the speed defines how often we sleep in between cycles.
    
    """
    wait, what if i didn't care about changing the rate of pulses and kept it at the minimum
    of 2.5 microseconds. they always move with a constant speed?

    That would require substantial changes to the stepper library. So maybe not.
    I will try it as is and go from there
    """
