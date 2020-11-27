#!/usr/bin/env python3

import time
import json
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from move import BLUE, RED, GREEN, YELLOW, ORANGE

def pixel_to_meters(j1, j2, length):
    return length / np.sqrt(np.sum((j1 - j2)**2))


class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera2 to a topic named image_topic2
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    self.template = cv2.imread("sphere_tmplt.png", 0)
    self.target_pub = rospy.Publisher('target_est', String, queue_size=10)
    self.joints_pub = rospy.Publisher('joints_est', String, queue_size=10)

    # need this to move joints 
    self.joints = {
            'joint_1': {
                'pos': np.array([0.0, 0.0, 0.0]),
                'colour': YELLOW
            },
            'joint_3': {
                'pos': np.array([0.0, 0.0, 0.0]),
                'colour': BLUE,
                },
            'joint_4': {
                'pos': np.array([0.0, 0.0, 0.0]),
                'colour': GREEN,
                },
            'end_effector': {
                'pos': np.array([0.0, 0.0, 0.0]),
                'colour': RED,
                }
            }

  # detect the joints
  def detect_joint_pos(self, image, joint_name):
    colour = self.joints[joint_name]['colour']
    prev_pos = np.array(self.joints[joint_name]['pos'])

    mask = cv2.inRange(image, colour[0], colour[1])
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)

    if M['m00'] == 0:
        # joint is hidden behind another joint. Return the last known position
        # and let the other script figure it out
        return np.array([0.0, prev_pos[0], prev_pos[1]]) 
    cx = float(M['m10'] / M['m00'])
    cy = float(M['m01'] / M['m00'])

    self.joints[joint_name]['pos'] = np.array([0.0, cx, cy])
    return np.array([0.0, cx, cy])

    
  def detect_target(self, image):
    # isolate the orange objects
    mask = cv2.inRange(image, ORANGE[0], ORANGE[1])
    # remove everything else (all else goes black)
    res = cv2.bitwise_and(image, image, mask=mask)
    # convert to greyscale 
    grey = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    # match with template of orange sphere 
    # can we use this method? Is it cheating?
    res = cv2.matchTemplate(grey, self.template, cv2.TM_CCOEFF_NORMED)

    _, _, _, max_loc = cv2.minMaxLoc(res)

    return np.array([0.0, max_loc[0], max_loc[1]]) # this is the centroid! 

  # Recieve data, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)
    im2=cv2.imshow('window2', self.cv_image2)
    cv2.waitKey(1)

    timestamp = rospy.get_rostime()

    j1_pos = self.detect_joint_pos(self.cv_image2, 'joint_1')
    j3_pos = self.detect_joint_pos(self.cv_image2, 'joint_3')
    j4_pos = self.detect_joint_pos(self.cv_image2, 'joint_4')
    ee_pos = self.detect_joint_pos(self.cv_image2, 'end_effector')
    target_pos = self.detect_target(self.cv_image2)

    a = pixel_to_meters(j1_pos, j3_pos, 2.5)
    j3_pos = a*(j1_pos - j3_pos)
    j4_pos = a*(j1_pos - j4_pos)
    ee_pos = a*(j1_pos - ee_pos)
    target_pos = a*(j1_pos-target_pos)

    joint_json = json.dumps({
        'projection': 'yz',
        'time': {
            'secs': timestamp.secs,
            'nsecs': timestamp.nsecs
            },
        'j1': j1_pos.tolist(),
        'j23': j3_pos.tolist(),
        'j4': j4_pos.tolist(),
        'ee': ee_pos.tolist(),
        'target': target_pos.tolist(),
        })

    target_json = json.dumps({
        'projection': 'yz',
        'time': {
            'secs': timestamp.secs,
            'nsecs': timestamp.nsecs
            },
        'pos': target_pos.tolist(),
        })

    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.target_pub.publish(String(target_json))
      self.joints_pub.publish(String(joint_json))
    except CvBridgeError as e:
      print(e)

def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
