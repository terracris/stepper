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

        self.home()


    def home(self):
        for joint in self.joints:
            joint.home()


