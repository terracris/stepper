import time
import ctypes
import numpy as np
from math import sqrt
import Jetson.GPIO as GPIO
from collections import deque

"""
Default motor rotation direction is CCW. However, you can set the motor to be inverted
"""
GPIO.setmode(GPIO.BOARD)

class Stepper:
    CCW = 1
    CW = 0
    libc = ctypes.CDLL("libc.so.6") # Load the C library
    MILLISECONDS_IN_SECOND = 1000    # number of milliseconds in one second

    """
    All motors use CCW as positive direction for rotation.
    max_speed comes in as pulses per second.
    """
    def __init__(self, pulse_pin, dir_pin, enable_pin, homing_pin, steps_per_rev, gear_ratio, max_speed, max_joint_ccw, max_joint_cw, home_count, inverted=False, kp=0.005, kd=0.003, has_homed = False):
        self.pulse_pin = pulse_pin
        self.dir_pin = dir_pin
        self.enable_pin = enable_pin
        self.homing_pin = homing_pin
        self.inverted = inverted # changes the positive direction for rotation
        self.direction = Stepper.CW if inverted else Stepper.CCW # current direction motor is spinning
        self.homing_direction = Stepper.CW if inverted else Stepper.CCW  # default homing direction is CCW
        self.positive_direction = Stepper.CW if inverted else Stepper.CCW # defualt positive direction is CCW
        self.negative_direction = Stepper.CCW if inverted else Stepper.CW # default negative direction is CW

        self.current_pos = 0
        self.target_pos = 0
        self.current_angle = 0

        # The current motor speed in steps per second --> Positive means counter-clockwise
        self.speed = 0
        self.max_speed = max_speed / Stepper.MILLISECONDS_IN_SECOND # our code operates in pulses per millisecond for increased accuracy
        self.acceleration = 0.0 # default to zero until set
        self.min_pulse_width = 2.5 # the minimum pulse width in microseconds (based on stepper driver)
        self.min_enable_setup = 0.2 # the minimum enable time in seconds (200 ms)
        self.min_dir_width = 5     # the minimum pulse width in microseconds (based on stepper driver)
        self.kp = kp
        self.kd = kd

        # gear ratio
        self.gear_ratio = gear_ratio
        self.steps_per_rev = steps_per_rev
        self.step_angle = 360 / steps_per_rev
        self.max_joint_limit_ccw = max_joint_ccw
        self.max_joint_limit_cw = max_joint_cw
        self.set_output_pins() # set up pins --> direction, pulse, enable
        self.has_homed = has_homed
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

        
        print("steps to travel to", absolute_steps)
        
        if absolute_steps > 0:
            self.direction = self.positive_direction # again positive steps are CCW
        else:
            self.direction = self.negative_direction
        
        self.move_absolute_pid(absolute_steps)
        
        # calculates our angle based on our current position
        # this accounts for angle errors
        self.current_angle = self.update_angle()

        return self.current_angle

    
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
                # print(self.current_pos, v_t, self.direction)
                self.step()
                start_time = Stepper.get_time()
                prev_error = error
                prev_v_t = v_t


    def get_distance_to_target(self):
        return self.target_pos - self.current_pos
    
    
    def step(self):
        self.set_direction_pins()
        GPIO.output(self.pulse_pin, GPIO.HIGH)
        Stepper.usleep(self.min_pulse_width)
        GPIO.output(self.pulse_pin, GPIO.LOW)
        Stepper.usleep(self.min_pulse_width)
        
        if self.has_homed:
            self.update_position()

    def update_position(self):
        if self.direction == self.positive_direction:
            self.current_pos += 1
        else:
            self.current_pos -= 1
    
    def set_direction_pins(self):
        if self.direction == Stepper.CCW:
            # print("ccw")
            GPIO.output(self.dir_pin, GPIO.HIGH) # When direction pin is HIGH, the motor will spin CCW
        else:
            # print("cw")
            GPIO.output(self.dir_pin, GPIO.LOW) # when direction pin is LOW, the motor will spin CW

        Stepper.usleep(self.min_dir_width)


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
        time.sleep(self.min_enable_setup) # minimum enable time is 200 ms 
    
    
    def home(self):
        
        is_home = GPIO.input(self.homing_pin)  # reads from homing pin to determine if we are home yet

        
        self.direction = self.homing_direction # all motors home in the CCW direction
        while not is_home:
            # if we are not home, we must rotate in our negative direction
            # until the limit switch has been hit
            
            is_home = GPIO.input(self.homing_pin)
            
            if is_home:
                break
            
            # print("homing direction is: ", self.direction)
            self.step()
            time.sleep(0.01) # sleep 10 ms between pulses --> gives pretty good speed

        self.stop()

        # once we have hit the limit switch, we must go to our home configuration step count
        # unfortunately, I believe this will have to be calculated experimentally. 
        # to minimize the error, we should increase the pulse number
        self.has_homed = True
        time.sleep(0.05) # wait to slow down completely
        home_count = self.home_count # count to home position
        self.direction = self.negative_direction  # we need to move in the opposite to our homing direction
        self.move_absolute_pid(home_count) # move there

        # after all homing is complete, we need to reset our position
        self.reset_position()

    def reset_position(self):
        self.current_pos = 0
        self.target_pos = 0
        self.current_angle = 0

    def move_clockwise(self, angle):
        steps = self.calculate_steps(angle)
        #print("number of steps: ",steps)
        self.direction = Stepper.CW
        #print("direction is: ",self.direction)
        self.set_direction_pins()
        # print("time for PID")
        time.sleep(0.5)
        self.move_absolute_pid(steps)
    
    def stop(self):
        GPIO.output(self.pulse_pin, GPIO.LOW)
        Stepper.usleep(self.min_pulse_width)
    
    def get_angle(self):
        return self.current_angle
    
    def cleanup(self):
        GPIO.cleanup()

    
    @staticmethod
    def usleep(microseconds):
        """
        Sleep for the given number of microseconds.
        """ 
        Stepper.libc.usleep(int(microseconds))


if __name__ == '__main__':

    # joint 3
    pulse_pin_j3 = 29
    dir_pin_j3 = 31
    homing_pin_j3 = 33
    gear_ratio_j3 = 4 * 5.18  # TODO review gear ratio
    home_count_j3 = -695  # TODO calculate home count
    max_speed_j3 = 50
    # gonna need to update kinematics to account for the joint limits:
    # like if it says j2 goes to 30 degrees, need to find clockwise alternative for all joints
    max_ccw_j3 = 90  # TODO calculate joint limit
    max_cw_j3 = -90  # TODO calculate joint limito
    pulses_per_rev = 200

     # joint 2
    pulse_pin_j2 = 19
    dir_pin_j2 = 21
    homing_pin_j2 = 23
    gear_ratio_j2 = 5 * 5.18
    home_count_j2 = -145
    max_speed_j2 = 50
    # gonna need to update kinematics to account for the joint limits:
    # like if it says j2 goes to 30 degrees, need to find clockwise alternative for all joints
    max_ccw_j2 = 135
    max_cw_j2 = -10

    # joint 4
    pulse_pin_j4 = 32
    dir_pin_j4 = 38
    homing_pin_j4 = 40
    gear_ratio_j4 = 1 # TODO calculate gear ratio
    home_count_j4 = -30 # TODO calculate home count
    max_speed_j4 = 5
    # gonna need to update kinematics to account for the joint limits:
    # like if it says j2 goes to 30 degrees, need to find clockwise alternative for all joints
    max_ccw_j4 = 90 # TODO calculate joint limits
    max_cw_j4 = -40 # TODO calcylate joint limit

    j2 = Stepper(pulse_pin_j2, dir_pin_j2, 12, homing_pin_j2, pulses_per_rev, gear_ratio_j2, max_speed_j2, max_ccw_j2, max_cw_j2, home_count_j2,inverted=True, has_homed=True)

    j3 = Stepper(pulse_pin_j3, dir_pin_j3, 12, homing_pin_j3, pulses_per_rev, gear_ratio_j3, max_speed_j3, max_ccw_j3, max_cw_j3, home_count_j3,kp=0.10,kd=0.003, has_homed=True)

    j4 = Stepper(pulse_pin_j4, dir_pin_j4, 12, homing_pin_j4, pulses_per_rev, gear_ratio_j4, max_speed_j4, max_ccw_j4, max_cw_j4, home_count_j4, kp=1, kd=0.003, has_homed=True)
    
    j4.write(45)

