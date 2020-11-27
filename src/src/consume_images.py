#!/usr/bin/env python3
""" 
consume streams from images
"""

import time
import json
import rospy
import threading
import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
from IVR_DH import get_htm
from IVR_Jacobian import get_jacobian
from IVR_Closed_PD import control_closed
import move


def pixel_to_meter(j1, j2, link_length): 
    dist = np.sum(np.abs(j1-j2)**2)
    return link_length / np.sqrt(dist)

def get_angles(j1, j2, axis='x'):

    # want to return 2 slices, the time stamps, and the angles at each timestamp
    # get list of timestamps

    if axis == 'x':
        ind = 0
    elif axis == 'y': 
        ind = 1
    elif axis == 'z':
        ind = 2
    else:
        return

    ay = np.array(j2)-np.array(j1)
    theta = np.arccos(ay[ind]/np.sqrt(np.sum(ay**2)))-np.pi/2
    return theta


class Consumer:
    def __init__(self):
        rospy.init_node("camera_consumers", anonymous=True)

        self.joint_sub = rospy.Subscriber('joints_est', String, self.callback)
        self.joint_pub = rospy.Publisher('joints_ang_est', JointState, queue_size=10)

        self.ee_pub_x = rospy.Publisher('ee_x_est', Float64 , queue_size=10)
        self.ee_pub_y = rospy.Publisher('ee_y_est', Float64 , queue_size=10)
        self.ee_pub_z = rospy.Publisher('ee_z_est', Float64 , queue_size=10)
        
        # self.target_pub = rospy.Publisher('target_pos_est', JointState, queue_size=10)
        self.prev_msg = {}
        self.prev_time = rospy.get_rostime().nsecs/1e9
        self.error = 0.0 # initial error?
        # need this to move the joints 
        self.mover = move.Mover()

    def perform_task_2(self, angles, ee_pos, target_pos, current_time):
        # dh = get_htm(angles)
        jac = get_jacobian(angles)
        dt = current_time - self.prev_time
        self.error, new_angles = control_closed(dt, ee_pos, target_pos, self.error, angles, jac)

        # publish new angles 
        self.mover.move_to(new_angles)

    def callback(self, data):
        data = json.loads(data.data)
        if self.prev_msg != {} and data['time']['nsecs'] - self.prev_msg['time']['nsecs'] < 1e7: # 10 ms
            if self.prev_msg['projection'] == 'xz':
                ind = 1
            elif self.prev_msg['projection'] == 'yz':
                ind = 0

            self.prev_msg['j1'][ind] = data['j1'][ind]
            self.prev_msg['j23'][ind] = data['j23'][ind]
            self.prev_msg['j4'][ind] = data['j4'][ind]
            self.prev_msg['ee'][ind] = data['ee'][ind]
            self.prev_msg['target'][ind] = data['target'][ind]

            # calculate angles:
            angles = [
                    np.arctan2(self.prev_msg['j4'][1], self.prev_msg['j4'][0]),
                    get_angles(self.prev_msg['j23'], self.prev_msg['j4'], 'x'),
                    get_angles(self.prev_msg['j23'], self.prev_msg['j4'], 'y'),
                    get_angles(self.prev_msg['j4'], self.prev_msg['ee'], 'x')
                    ]

            self.ee_pub_x.publish(Float64(self.prev_msg['ee'][0]))
            self.ee_pub_y.publish(Float64(self.prev_msg['ee'][1]))
            self.ee_pub_z.publish(Float64(self.prev_msg['ee'][2]))

            # A = get_htm(angles)
            # print(angles)
            # print([A[0,3], A[1,3], A[2,3]])
            self.perform_task_2(
                    angles, 
                    np.array(self.prev_msg['ee']), 
                    np.array(self.prev_msg['target']), 
                    self.prev_msg['time']['nsecs']/1e9
                )
            self.prev_time = data['time']['nsecs']/1e9

            msg = JointState()
            msg.position = angles
            msg.header.stamp.secs = self.prev_msg['time']['secs']
            msg.header.stamp.nsecs = self.prev_msg['time']['nsecs']
            self.joint_pub.publish(msg)
            self.prev_msg = {}
        else:
            self.prev_msg = data
if __name__ == "__main__":
    c = Consumer()
    start_time = time.time()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

