#!/usr/bin/env python3

import time
import rospy
import json
import threading
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt


class TargetReader:
    def __init__(self):
        rospy.init_node("target_position_estimation", anonymous=True)

        self.lock = threading.Lock()
        self.target_est_x = [[], []]
        self.target_est_y = [[], []]
        self.target_est_z = [[], []]

        self.target_pos_x = [[], []]
        self.target_pos_y = [[], []]
        self.target_pos_z = [[], []]

        self.target_est_sub = rospy.Subscriber(
            "target_est",
            String,
            self.est_callback
        )

        self.target_pos_x_sub = rospy.Subscriber(
            "target/joint_states",
            JointState,
            self.target_pos_callback
        )

    def target_pos_callback(self, data):
        time = data.header.stamp.secs
        self.target_pos_x[0].append(time)
        self.target_pos_x[1].append(data.position[0])
        self.target_pos_y[0].append(time)
        self.target_pos_y[1].append(data.position[1])
        self.target_pos_z[0].append(time)
        self.target_pos_z[1].append(data.position[2])


    def est_callback(self, data):
        data = json.loads(data.data)

        if data['projection'] == 'xz':
            self.target_est_x[0].append(np.float64(data['time']['secs']))
            self.target_est_x[1].append(data['pos'][0])
        elif data['projection'] == 'yz':
            print("here!!")
            self.target_est_y[0].append(np.float64(data['time']['secs']))
            self.target_est_y[1].append(data['pos'][1])
        # if the time stamp is close enough to the previous addition, take
        # the average of the two as they correspond to the same reading
        # but from different projections
            self.target_est_z[0].append(np.float64(data['time']['secs']))
            self.target_est_z[1].append(data['pos'][2])

    def make_plots(self):
        plt.figure()
        plt.plot(
            self.target_est_x[0],
            self.target_est_x[1],
            'b-',
            label="estimated x")
        plt.plot(
            self.target_pos_x[0],
            self.target_pos_x[1],
            'r-',
            label="actual x")
        plt.legend()
        plt.xlabel("time")
        plt.ylabel("x coordinate")
        plt.show()

        plt.figure()
        plt.plot(
            self.target_est_y[0],
            self.target_est_y[1],
            'b-',
            label="estimated y")
        plt.plot(
            self.target_pos_y[0],
            self.target_pos_y[1],
            'r-',
            label="actual y")
        plt.legend()
        plt.xlabel("time")
        plt.ylabel("y coordinate")
        plt.show()

        plt.figure()
        plt.plot(
            self.target_est_z[0],
            self.target_est_z[1],
            'b-',
            label="estimated z")
        plt.plot(
            self.target_pos_z[0],
            self.target_pos_z[1],
            'r-',
            label="actual z")
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
