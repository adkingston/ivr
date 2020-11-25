"""
class to move the joints as per the specifications of question 1
"""
import time
import numpy as np
import rospy
from std_msgs.msg import Float64

# useful constants
BLUE = [(100, 0, 0), (255, 0, 0)]
GREEN = [(0, 100, 0), (0, 255, 0)]
RED = [(0, 0, 100), (0, 0, 255)]
YELLOW = [(0, 100, 100), (0, 255, 255)]
ORANGE = [(5, 50, 50), (15, 255, 255)]



class Mover:
    """ implements the joint movements per joint for question 1 """
    def __init__(self):
        self.start = time.time()
        self.joint2_pub = rospy.Publisher(
                "/robot/joint2_position_controller/command",
                Float64, queue_size=10
                )
        self.joint3_pub = rospy.Publisher(
                "/robot/joint3_position_controller/command",
                Float64, queue_size=10
                )
        self.joint4_pub = rospy.Publisher(
                "/robot/joint4_position_controller/command",
                Float64, queue_size=10
                )

        self.joint2_angle = np.pi/15
        self.joint3_angle = np.pi/18
        self.joint4_angle = np.pi/20

    def next_position(self, angle, t):
        """ calculate the next position given the angle scalar of a joint """
        return Float64((np.pi/2)*np.sin(angle*t))

    def move(self):
        """ publish the new angles for each joint """
        t = time.time()
        self.joint2_pub.publish(self.next_position(self.joint2_angle, t))
        self.joint3_pub.publish(self.next_position(self.joint3_angle, t))
        self.joint4_pub.publish(self.next_position(self.joint4_angle, t))
