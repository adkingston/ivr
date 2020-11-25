#!/usr/bin/env python3
""" 
consume streams from images
"""

import time
import rospy
import threading
import numpy as np
import numpy.linalg as la
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from std_msgs.msg import Float64MultiArray
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

def get_angles(j1, j2, link_length, axis='x'):

    # want to return 2 slices, the time stamps, and the angles at each timestamp
    # get list of timestamps

    r1 = [t[1] for t in j1]
    r2 = [t[1] for t in j2]

    if axis == 'x':
        ind = 0
    elif axis == 'y': 
        ind = 1
    elif axis == 'z':
        ind = 2
    else:
        return

    domain = []
    ran = []
    for i in range(min(len(r1), len(r2))):
        domain.append(j1[i][0])
        x, y = r1[i], r2[i]
        ay = (y - x)
        theta = np.arccos(ay[ind]/np.sqrt(np.sum(ay**2)))-np.pi/2


        ran.append(theta)

    return domain, ran

def joint1_angle(positions):
    r = [t[1] for t in positions]
    domain = [t[0] for t in positions]
    ran = []
    for i in range(len(r)):
        pos = r[i]
        ran.append(np.arctan2(pos[1], pos[0]))

    return domain, ran

class Consumer:
    def __init__(self):
        rospy.init_node("camera_consumers", anonymous=True)
        # multiple threads are making changes to this class
        # use a lock to prevent race conditions 
        self.lock = threading.Lock()
        # only need actual. We will use the angle of link 2 about the z axis 
        self.joint1_pos_act = [[],[]] 

        self.joint23_pos_est = []
        self.joint2_pos_act = [[],[]] # times, angles
        self.joint3_pos_act = [[],[]]

        self.joint4_pos_est = []
        self.joint4_pos_act = [[],[]]

        self.ee_pos_est = []

        self.joint2_sub = rospy.Subscriber(
                "joint2_topic", 
                Float64MultiArray, 
                self.callback2)
        self.joint3_sub = rospy.Subscriber(
                "joint3_topic", 
                Float64MultiArray, 
                self.callback2)
        self.joint4_sub = rospy.Subscriber(
                "joint4_topic", 
                Float64MultiArray, 
                self.callback4)
        self.ee_sub = rospy.Subscriber(
                "ee_topic", 
                Float64MultiArray, 
                self.callback_ee)

        self.act_angle_sub = rospy.Subscriber(
                "robot/joint_states",
                JointState,
                self.callback_actual_pos
                )
    
    def callback2(self, data):
        self.lock.acquire()
        add = merge_streams(data, self.joint23_pos_est)
        self.lock.release()


    def callback4(self, data):
        self.lock.acquire()
        add = merge_streams(data, self.joint4_pos_est)
        self.lock.release()


    def callback_ee(self, data):
        self.lock.acquire()
        add = merge_streams(data, self.ee_pos_est)
        self.lock.release()
        return

    def callback_actual_pos(self, data):
        self.lock.acquire()
        timestamp = time.time()
        # print(data.position)
        self.joint1_pos_act[0].append(timestamp)
        self.joint1_pos_act[1].append(data.position[0])
        self.joint2_pos_act[0].append(timestamp)
        self.joint2_pos_act[1].append(data.position[1])
        self.joint3_pos_act[0].append(timestamp)
        self.joint3_pos_act[1].append(data.position[2])
        self.joint4_pos_act[0].append(timestamp)
        self.joint4_pos_act[1].append(data.position[3])

        self.lock.release()



if __name__ == "__main__":
    c = Consumer()
    start_time = time.time()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")


    times, est_angle_j1 = joint1_angle(c.joint4_pos_est)
    plt.figure()
    plt.plot(times, est_angle_j1, 'b-', label="joint1 estimate")
    plt.plot(c.joint1_pos_act[0], c.joint1_pos_act[1], 'r-', label="joint1 actual")
    # plt.plot(times, c.ee_pos_act, 'g-')
    plt.legend()
    plt.xlabel("time (seconds)")
    plt.ylabel("angle (radians)")
    plt.show()

    times, est_angle_j2 = get_angles(c.joint23_pos_est, c.joint4_pos_est, 3.5, axis='x')
    plt.figure()
    plt.plot(times, est_angle_j2, 'b-', label="joint2 estimate")
    plt.plot(c.joint2_pos_act[0], c.joint2_pos_act[1], 'r-', label="joint2 actual")
    # plt.plot(times, c.ee_pos_act, 'g-')
    plt.legend()
    plt.xlabel("time (seconds)")
    plt.ylabel("angle (radians)")
    plt.show()

    times, est_angle_j3 = get_angles(c.joint23_pos_est, c.joint4_pos_est, 3.5, axis='y')
    plt.figure()
    plt.plot(times, est_angle_j3, 'b-', label="joint3 estimate")
    plt.plot(c.joint3_pos_act[0], c.joint3_pos_act[1], 'r-', label="joint3 actual")
    plt.legend()
    plt.xlabel("time (seconds)")
    plt.ylabel("angle (radians)")
    plt.show()

    times, est_angle_j4 = get_angles(c.joint4_pos_est, c.ee_pos_est, 3, axis='x')
    plt.figure()
    plt.plot(times, est_angle_j4, 'b-', label="joint4 estimate")
    plt.plot(c.joint4_pos_act[0], c.joint4_pos_act[1], 'r-', label="joint4 actual")
    plt.legend()
    plt.xlabel("time (seconds)")
    plt.ylabel("angle (radians)")
    plt.show()
    
