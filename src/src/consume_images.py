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
from std_msgs.msg import String
from sensor_msgs.msg import JointState

def merge_streams(data, stream_store):
   # if this point came in less than 0.05 seconds after the previous one
   if len(stream_store) > 0 and data.data[0] - stream_store[-1][0] < 0.1:
       if data.layout.dim[1].label == 'y':
           stream_store[-1] = [
                   stream_store[-1][0], 
                   np.array([
                       stream_store[-1][1][0], 
                       data.data[1], 
                       float((data.data[2] + stream_store[-1][1][2])/2)
                   ])
               ]
       else:
           stream_store[-1] = [
                   stream_store[-1][0], 
                   np.array([
                       data.data[1], 
                       stream_store[-1][1][1], 
                       float((data.data[2] + stream_store[-1][1][2])/2)
                   ])
               ]
       return False

   else: 
       if data.layout.dim[1].label == 'y':
           stream_store.append([np.round(data.data[0], 2), np.array([0.0, data.data[1], data.data[2]])])
       else:
           stream_store.append([np.round(data.data[0], 2), np.array([data.data[1], 0.0, data.data[2]])])

       return True


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
        self.prev_msg = {}

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

            # calculate angles:
            angles = [
                    -np.arctan2(self.prev_msg['j4'][1], self.prev_msg['j4'][0]),
                    get_angles(self.prev_msg['j23'], self.prev_msg['j4'], 'x'),
                    get_angles(self.prev_msg['j23'], self.prev_msg['j4'], 'y'),
                    get_angles(self.prev_msg['j4'], self.prev_msg['ee'], 'x')
                    ]

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

