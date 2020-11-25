#!/usr/bin/env python3

import time
import rospy
import threading
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
import matplotlib.pyplot as plt

class TargetReader:
    def __init__(self):
        rospy.init_node("target_position_estimation", anonymous=True)

        self.lock = threading.Lock()
        self.target_est_x = [[],[]]
        self.target_est_y = [[],[]]
        self.target_est_z = [[],[]]

        self.target_pos_x = [[],[]]
        self.target_pos_y = [[],[]]
        self.target_pos_z = [[],[]]

        self.target_est_sub = rospy.Subscriber(
                "target_est",
                Float64MultiArray,
                self.est_callback
                )

        self.target_pos_x_sub = rospy.Subscriber(
                "target/x_position_controller/command",
                Float64,
                self.target_pos_callback(self.target_pos_x)
                )
        self.target_pos_y_sub = rospy.Subscriber(
                "target/y_position_controller/command",
                Float64,
                self.target_pos_callback(self.target_pos_y)
                )
        self.target_pos_z_sub = rospy.Subscriber(
                "target/z_position_controller/command",
                Float64,
                self.target_pos_callback(self.target_pos_z)
                )

    def target_pos_callback(self, store):
        def callback(data):
            store[0].append(np.round(time.time(), 2))
            store[1].append(data.data)
        
        return callback

    def est_callback(self, data):
        if data.layout.dim[1].label == 'x':
            self.target_est_x[0].append(np.round(data.data[0], 2))
            self.target_est_x[1].append(data.data[1])
        elif data.layout.dim[1].label == 'y':
            print("here!!")
            self.target_est_y[0].append(np.round(data.data[0], 2))
            self.target_est_y[1].append(data.data[1])
        # if the time stamp is close enough to the previous addition, take 
        # the average of the two as they correspond to the same reading 
        # but from different projections 
            self.target_est_z[0].append(np.round(data.data[0], 2))
            self.target_est_z[1].append(data.data[2])

    def make_plots(self):
        plt.figure()
        plt.plot(self.target_est_x[0], self.target_est_x[1], 'b-', label="estimated x")
        plt.plot(self.target_pos_x[0], self.target_pos_x[1], 'r-', label="actual x")
        plt.legend()
        plt.xlabel("time")
        plt.ylabel("x coordinate")
        plt.show()

        plt.figure()
        plt.plot(self.target_est_y[0], self.target_est_y[1], 'b-', label="estimated y")
        plt.plot(self.target_pos_y[0], self.target_pos_y[1], 'r-', label="actual y")
        plt.legend()
        plt.xlabel("time")
        plt.ylabel("y coordinate")
        plt.show()

        plt.figure()
        plt.plot(self.target_est_z[0], self.target_est_z[1], 'b-', label="estimated z")
        plt.plot(self.target_pos_z[0], self.target_pos_z[1], 'r-', label="actual z")
        plt.legend()
        plt.xlabel("time")
        plt.ylabel("z coordinate")
        plt.show()

def main():
    t = TargetReader()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")

    t.make_plots()
        
if __name__ == "__main__":
    main()
