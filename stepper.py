import time
import ctypes
import numpy as np
from math import sqrt
import Jetson.GPIO as GPIO
from collections import deque

class Stepper:
    CCW = 1
    CW = 0
    libc = ctypes.CDLL("libc.so.6") # Load the C library
    MILLISECONDS_IN_SECOND = 1000    # number of milliseconds in one second
    GPIO.setmode(GPIO.BOARD)

    """
    Each motor can have a defined positive direction for rotation
    """
    def __init__(self, pulse_pin, dir_pin, enable_pin, homing_pin, steps_per_rev, gear_ratio, positive_direction, max_joint_ccw, max_joint_cw, home_count):
        self.pulse_pin = pulse_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.homing_pin = homing_pin
        self.direction = Stepper.CCW if (positive_direction == 1) else Stepper.CW # current direction motor is spinning ini
        self.positive_direction = Stepper.CCW if (positive_direction == 1) else Stepper.CW
        self.negative_direction = Stepper.CW if (positive_direction == 1) else Stepper.CCW


        self.current_pos = 0
        self.target_pos = 0
        # The current motor speed in steps per second --> Positive means counter-clockwise
        self.speed = 0
        self.max_speed = 0.0 # default to zero until set
        self.acceleration = 0.0 # default to zero until set
        self.step_interval = 0 # the current interval between steps in microseconds (defines the angular velocity (RPM) and acceleration)
        self.min_pulse_width = 2.5 # the minimum pulse width in seconds (based on stepper driver)
        self.last_step_time = 0 # the time the last step was done

        self.current_angle = 0
        # gear ratio
        self.gear_ratio = gear_ratio
        self.steps_per_rev = steps_per_rev
        self.step_angle = 360 / steps_per_rev
        self.max_joint_limit_ccw = max_joint_ccw
        self.max_joint_limit_cw = max_joint_cw
        self.set_output_pins() # set up pins --> direction, pulse, enable
        self.has_homed = False
        self.home_count = home_count
        self.home()

   
    # write angles
    def write(self, angle):
        # check if the angle is within the limits for the joint
        # if it is, calculate the angles number of steps to get there
        
        if not self.in_limits(angle):
            print("angle out of limit")
            return
        
        # number of steps to go to from our current position
        steps = self.calculate_steps(angle)
        self.move_absolute_pid(self.current_pos + steps)
        # technically, because I round the value, there is an error in my actual angle
        # actually, I should account for this error because it will build up immensely
        # TODO correct for the error
        self.current_angle = self.update_angle()

    
    def in_limits(self, angle):
        return angle >= self.max_joint_limit_cw and angle <= self.max_joint_limit_ccw
    
    """
    converts pulse position to angle in degrees.
    """
    def update_angle(self):
        return (self.current_pos * self.step_angle) / self.gear_ratio

    
    """
    Converts desired angle to steps to control the stepper motor.
    angle: angle from -jointLimitCW to JointLimitCCW
    Changes the direction of rotation 
    """
    def calculate_steps(self, angle):
        current_angle = self.current_angle
        change_in_angle = angle - current_angle

        # if the change in angle is negative, we move in that motor's negative direction
        if change_in_angle < 0:
            print("negative direction", self.negative_direction)
            self.direction = self.negative_direction
        else:
            self.direction = self.positive_direction
        
        # rounds the number of steps required and then turns it into an int (just in case)
        goal_steps = int(round(change_in_angle * self.gear_ratio / self.step_angle))

        return goal_steps


    def move_absolute_pid(self, absolute):
        # if we are not there set the target position
        if self.current_pos != absolute:
            self.target_pos = absolute
        else:
            return
        
        Kp = 0.0050
        Kd = -0.003

        prev_error = 0  # Initialize prev_error outside the loop
        max_velocity = 10
        max_integral = 10
        start_time = self.get_time()
        max_acceleration = 0.20 # max pulses per ms per ms
        prev_v_t = 0

        while self.current_pos != absolute:
            error = self.get_distance_to_target()
            error_der = error - prev_error
            # print("error derivative: ", error_der) 

            v_t = abs(Kp * error + Kd * error_der)  # constrain maximum velocity
            
            # if v_t - prev_v_t >= max_acceleration:
               #  v_t = v_t / 2

            v_t =  v_t
            
            self.step_interval = 1 / v_t  # [ms period between each pulse]

            if self.get_time() - start_time >= self.step_interval:
                print(self.current_pos, v_t, self.direction)
                self.step()
                start_time = self.get_time()
                prev_error = error
                prev_v_t = v_t


    def get_distance_to_target(self):
        return self.target_pos - self.current_pos
    
    
    def step(self):
        self.set_direction_pins()
        GPIO.output(self.pulse_pin, GPIO.HIGH)
        Stepper.usleep(self.min_pulse_width) # TODO need to set minimum pulse width as 2.5 microseconds
        GPIO.output(self.pulse_pin, GPIO.LOW)
        if self.has_homed:
            self.update_position()

    def update_position(self):
        if self.direction == self.positive_direction:
            self.current_pos += 1
        else:
            self.current_pos -= 1
    
    def set_direction_pins(self):
        if self.direction == Stepper.CCW:
            print("ccw")
            GPIO.output(self.dir_pin, GPIO.HIGH) # When direction pin is HIGH, the motor will spin CCW
        else:
            GPIO.output(self.dir_pin, GPIO.LOW) # when direction pin is LOW, the motor will spin CW

   # returns time in microseconds since epoch 1970 
    def get_time(self):
        return int(time.time() * Stepper.MILLISECONDS_IN_SECOND)
   
    def set_output_pins(self):
        
        GPIO.setup(self.pulse_pin, GPIO.OUT)   # output pin 
        GPIO.setup(self.dir_pin, GPIO.OUT)     # output pin
        GPIO.setup(self.enable_pin, GPIO.OUT)  # output pin
        GPIO.setup(self.homing_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # input pin 
        
        GPIO.output(self.enable_pin, GPIO.LOW) # turn motors on
        GPIO.output(self.dir_pin, GPIO.HIGH)    # HIGH is ccw
        GPIO.output(self.pulse_pin, GPIO.LOW)  # no pulse 
    
    
    def home(self):
        
        is_home = GPIO.input(self.homing_pin)  # reads from homing pin to determine if we are home yet

        while not is_home:
            # if we are not home, we must rotate in our negative direction
            # until the limit switch has been hit
            print("homing")
            self.direction = self.negative_direction
            self.step()
            is_home = GPIO.input(self.homing_pin)
            time.sleep(0.01)

        # once we have hit the limit switch, we must go to our home configuration step count
        # unfortunately, I believe this will have to be calculated experimentally. 
        # to minimize the error, we should increase the pulse number
        self.has_homed = True
        time.sleep(1)
        home_count = self.home_count
        self.direction = self.positive_direction
        self.move_absolute_pid(home_count)

    
    @staticmethod
    def usleep(microseconds):
        """
        Sleep for the given number of microseconds.
        """ 
        Stepper.libc.usleep(int(microseconds))
   
if __name__ == '__main__':
   #  pulse_pin = 11
   # direction_pin = 15
   # homing_pin = 13
    pulses_per_rev = 200
    # gear_ratio = 4
    #positive_direction = 0 #CW
    #home_count = 340

    #motor = Stepper(pulse_pin,direction_pin,12,homing_pin, pulses_per_rev, gear_ratio, positive_direction, 210, -10, home_count)

    pulse_pin_1 = 31
    dir_pin_1 = 37
    homing_pin_1 = 33
    positive_direction_1 = 0 # CCW
    gear_ratio_1 = 5
    home_count_1 = 1000

    motor1 = Stepper(pulse_pin_1, dir_pin_1, 12, homing_pin_1, pulses_per_rev, gear_ratio_1, positive_direction_1, 210, -10, home_count_1)