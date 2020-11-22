#!/usr/bin/env python3
""" 
consume streams from images
"""

import time
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from std_msgs.msg import Float64MultiArray

class Consumer:
    def __init__(self):
        rospy.init_node("camera_consumers", anonymous=True)

        self.joint23_pos_est = []
        self.joint23_pos_act = []

        self.joint4_pos_est = []
        self.joint4_pos_act = []

        self.ee_pos_est = []
        self.ee_pos_act = []

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
    
    def callback2(self, data):
        # if this point came in less than 0.05 seconds after the previous one
        if len(self.joint23_pos_est) > 0 and data.data[0] - self.joint23_pos_est[-1][0] < 0.1:
            print(self.joint23_pos_est[-1][0] - data.data[0])
            if data.layout.dim[1].label == 'y':
                self.joint23_pos_est[-1] = [
                        self.joint23_pos_est[-1][0], 
                        [
                            self.joint23_pos_est[-1][1][0], 
                            data.data[1], 
                            int((data.data[2] + self.joint23_pos_est[-1][1][2])/2)
                        ]
                    ]
            else:
                self.joint23_pos_est[-1] = [
                        self.joint23_pos_est[-1][0], 
                        [
                            data.data[1], 
                            self.joint23_pos_est[-1][1][1], 
                            int((data.data[2] + self.joint23_pos_est[-1][1][2])/2)
                        ]
                    ]

        else: 
            if data.layout.dim[1].label == 'y':
                self.joint23_pos_est.append([data.data[0], [0.0, data.data[1], data.data[2]]])
            else:
                self.joint23_pos_est.append([data.data[0], [data.data[1], 0.0, data.data[2]]])

        self.joint23_pos_act.append((np.pi/2)*np.sin((np.pi/15)*data.data[0]))

        print(self.joint23_pos_est)
    # print(data.data)


    # def callback3(self, data):
        # print(f"joint2: {data.data}, projection: ({data.layout.dim[1].label, data.layout.dim[2].label})")
    
    def callback4(self, data):
        return
        # print(f"joint4: {data.data}, projection: ({data.layout.dim[1].label, data.layout.dim[2].label})")


    def callback_ee(self, data):
        return
        # print(f"ee: {data.data}, projection: ({data.layout.dim[1].label, data.layout.dim[2].label})")


def t():
    print("hello, world!")

if __name__ == "__main__":
    c = Consumer()
    start_time = time.time()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    X = [c.joint23_pos_est[x][1][0] for x in range(len(c.joint23_pos_est))]
    Y = [c.joint23_pos_est[x][1][1] for x in range(len(c.joint23_pos_est))]
    Z = [c.joint23_pos_est[x][1][2] for x in range(len(c.joint23_pos_est))]

    plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(X, Y, Z)
    plt.show()
    print("shutting down")

        
