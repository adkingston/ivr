"""
class to move the joints as per the specifications of question 1
"""
import numpy as np
import rospy
from std_msgs.msg import Float64


class Mover:
    """ implements the joint movements per joint for question 1 """
    def __init__(self):
        self.i = 0
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

    def next_position(self, angle):
        """ calculate the next position given the angle scalar of a joint """
        return Float64((np.pi/2)*np.sin(angle*self.i))

    def move(self):
        """ publish the new angles for each joint """
        self.joint2_pub.publish(self.next_position(self.joint2_angle))
        self.joint3_pub.publish(self.next_position(self.joint3_angle))
        self.joint4_pub.publish(self.next_position(self.joint4_angle))
        self.i += 1
