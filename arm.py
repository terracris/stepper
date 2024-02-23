import threading
from stepper import Stepper

class Arm:

    # I am going to make the arm take in 4 different motors on startup
    def __init__(self, j1, j2, j3, j4):
        
        self.j1 = j1
        self.j2 = j2
        self.j3 = j3
        self.j4 = j4
        self.joints = [j1, j2, j3, j4]
        self.j1_angle = 0
        self.j2_angle = 0
        self.j3_angle = 0
        self.j4_angle = 0
        self.threads = [] # empty list for threads for homing

        self.home()


    def home(self):
        # joint is the actual stepper motor class

        for joint in self.joints:
            thread = threading.Thread(target=self.home_joint, args=(joint))
            thread.start()
            self.threads.append(thread)

        # wait for each joint to home
        for thread in self.threads:
            thread.join()
        print("All threads finished")

    # blocking function to home each joint
    def home_joint(self, joint):
        joint.home()


if __name__ == '__main__':
    
    pulses_per_rev = 200
    
    # joint 1
    pulse_pin_j1 = 11
    dir_pin_j1 = 15
    homing_pin_j1 = 13
    gear_ratio_j1 = 4
    home_count_j1 = 340
    max_speed_j1 = 10
    max_ccw_j1 = 90
    max_cw_j1 = -90

    # joint 2
    pulse_pin_j2 = 31
    dir_pin_j2 = 37
    homing_pin_j2 = 33
    gear_ratio_j2 = 5
    home_count_j2 = -1000
    max_speed_j2 = 10
    max_ccw_j2 = 10
    max_cw_j2 = -135

    # joint 3

    # joint 4

    j1 =j2 = Stepper(pulse_pin_j1, dir_pin_j1, 12, homing_pin_j1, pulses_per_rev, gear_ratio_j1, max_speed_j1, max_ccw_j1, max_cw_j1, home_count_j1) 
    j2 = Stepper(pulse_pin_j2, dir_pin_j2, 12, homing_pin_j2, pulses_per_rev, gear_ratio_j2, max_speed_j2, max_ccw_j2, max_cw_j2, home_count_j2)