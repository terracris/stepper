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
    All motors use CCW as positive direction for rotation.
    max_speed comes in as pulses per second.
    """
    def __init__(self, pulse_pin, dir_pin, enable_pin, homing_pin, steps_per_rev, gear_ratio, max_speed, max_joint_ccw, max_joint_cw, home_count, kp=0.005, kd=0.003):
        self.pulse_pin = pulse_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.homing_pin = homing_pin
        self.direction = Stepper.CCW # current direction motor is spinning

        self.current_pos = 0
        self.target_pos = 0
        self.current_angle = 0

        # The current motor speed in steps per second --> Positive means counter-clockwise
        self.speed = 0
        self.max_speed = max_speed / Stepper.MILLISECONDS_IN_SECOND # our code operates in pulses per millisecond for increased accuracy
        self.acceleration = 0.0 # default to zero until set
        self.min_pulse_width = 2.5 # the minimum pulse width in seconds (based on stepper driver)
        self.kp = kp
        self.kd = kd

        # gear ratio
        self.gear_ratio = gear_ratio
        self.steps_per_rev = steps_per_rev
        self.step_angle = 360 / steps_per_rev
        self.max_joint_limit_ccw = max_joint_ccw
        self.max_joint_limit_cw = max_joint_cw
        self.set_output_pins() # set up pins --> direction, pulse, enable
        self.has_homed = False
        self.home_count = home_count
        # self.home()

   
    # write angles
    # sets the direction
    def write(self, angle):
        # check if the angle is within the limits for the joint
        # if it is, calculate the angles number of steps to get there
        
        if not self.in_limits(angle):
            print("angle out of limit")
            return
        
        # absolute step position for arm
        absolute_steps = self.calculate_steps(angle)
        
        if absolute_steps > 0:
            self.direction = Stepper.CCW # again positive steps are CCW
        else:
            self.direction = Stepper.CW
        
        self.move_absolute_pid(absolute_steps)
        
        # calculates our angle based on our current position
        # this accounts for angle errors
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

    negative goal steps indicates move CW. positive goal steps are CCW
    """
    def calculate_steps(self, angle):
        
        # rounds the number of steps required and then turns it into an int (just in case)
        goal_steps = int(round(angle * self.gear_ratio / self.step_angle))

        return goal_steps


    def move_absolute_pid(self, absolute):
        # if we are not there set the target position
        if self.current_pos != absolute:
            self.target_pos = absolute
        else:
            return
        
        Kp = self.kp
        Kd = self.kd

        prev_error = 0  # Initialize prev_error outside the loop
        start_time = Stepper.get_time()
        max_acceleration = 0.20 # max pulses per ms per ms
        prev_v_t = 0

        while self.current_pos != absolute:
            error = self.get_distance_to_target()
            error_der = error - prev_error
            # print("error derivative: ", error_der) 

            v_t = abs(Kp * error + Kd * error_der)  # constrain maximum velocity
            v_t =  min(self.max_speed, v_t)
            
            self.step_interval = 1 / v_t  # [ ms period between each pulse ]

            if Stepper.get_time() - start_time >= self.step_interval:
                print(self.current_pos, v_t, self.direction)
                self.step()
                start_time = Stepper.get_time()
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
        if self.direction == Stepper.CCW:
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
    @staticmethod 
    def get_time():
        return int(time.time() * Stepper.MILLISECONDS_IN_SECOND)
   
    def set_output_pins(self):
        
        GPIO.setup(self.pulse_pin, GPIO.OUT)   # output pin 
        GPIO.setup(self.dir_pin, GPIO.OUT)     # output pin
        GPIO.setup(self.enable_pin, GPIO.OUT)  # output pin
        GPIO.setup(self.homing_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # input pin 
        
        GPIO.output(self.enable_pin, GPIO.LOW) # turn motors on
        GPIO.output(self.dir_pin, GPIO.HIGH)    # HIGH is ccw --> default direction
        GPIO.output(self.pulse_pin, GPIO.LOW)  # no pulse 
    
    
    def home(self):
        
        is_home = GPIO.input(self.homing_pin)  # reads from homing pin to determine if we are home yet

        
        self.direction = Stepper.CCW  # all motors home in the CCW direction
        while not is_home:
            # if we are not home, we must rotate in our negative direction
            # until the limit switch has been hit
            
            is_home = GPIO.input(self.homing_pin)
            
            if is_home:
                break
            
            print("homing direction is: ", self.direction)
            self.step()
            time.sleep(0.01)

        self.stop()

        # once we have hit the limit switch, we must go to our home configuration step count
        # unfortunately, I believe this will have to be calculated experimentally. 
        # to minimize the error, we should increase the pulse number
        self.has_homed = True
        time.sleep(1) # wait to slow down completely
        # home_count = self.home_count # count to home position
        # self.direction = Stepper.CW  # we need to move in the opposite to our homing direction
        # self.move_absolute_pid(home_count) # move there

        self.move_clockwise(-20)
        
        # after all homing is complete, we need to reset our position
        self.reset_position()

    def reset_position(self):
        self.current_pos = 0
        self.target_pos = 0
        self.current_angle = 0

    def move_clockwise(self, angle):
        steps = self.calculate_steps(angle)
        print("number of steps: ",steps)
        self.direction = Stepper.CW
        print("direction is: ",self.direction)
        self.set_direction_pins()
        print("time for PID")
        time.sleep(0.5)
        self.move_absolute_pid(steps)
    
    def stop(self):
        GPIO.output(self.pulse_pin, GPIO.LOW)
    
    def get_angle(self):
        return self.current_angle

    
    @staticmethod
    def usleep(microseconds):
        """
        Sleep for the given number of microseconds.
        """ 
        Stepper.libc.usleep(int(microseconds))

if __name__ == '__main__':
    pulses_per_rev = 200

    # joint 1
    pulse_pin_j1 = 32
    dir_pin_j1 = 22
    homing_pin_j1 = 40
    gear_ratio_j1 = 5 * 5.18
    home_count_j1 = -200
    max_speed_j1 = 50
    max_ccw_j1 = 90
    max_cw_j1 = -90
    
    j1 = Stepper(pulse_pin_j1, dir_pin_j1, 12, homing_pin_j1, pulses_per_rev, gear_ratio_j1, max_speed_j1, max_ccw_j1, max_cw_j1, home_count_j1, 0.05, 0.03)
    print("about to move")
    # j1.move_clockwise(-20)
    j1.home()
    
    print("done")
    while True:
        pass
