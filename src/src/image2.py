#!/usr/bin/env python3

import time
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError
from move import BLUE, RED, GREEN, YELLOW

class Dim: 
    def __init__(self, label):
        self.label = label
        self.size = 1
        self.stride = 1


class Layout:
    def __init__(self):
        self.dim = [Dim('time'), Dim('x'), Dim('z')]
        self.data_offset = 0

LAYOUT = Layout()

def pixel_to_meters(j1, j2, length):
    return length/np.sqrt(np.sum(np.abs(j1-j2)**2))

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

    # need this to move joints 
    self.joints = {
            'joint_1': {
                'pos': np.array([0.0, 0.0]),
                'colour': YELLOW
                },
            'joint_3': {
                'pos': np.array([0.0, 0.0]),
                'colour': BLUE,
                'pub': rospy.Publisher('joint3_topic', Float64MultiArray, queue_size=10)
                },
            'joint_4': {
                'pos': np.array([0.0, 0.0]),
                'colour': GREEN,
                'pub': rospy.Publisher('joint4_topic', Float64MultiArray, queue_size=10)
                },
            'end_effector': {
                'pos': np.array([0.0, 0.0]),
                'colour': RED,
                'pub': rospy.Publisher('ee_topic', Float64MultiArray, queue_size=10)
                }
            }


  # detect the joints
  def detect_joint_pos(self, image, joint_name):
    colour = self.joints[joint_name]['colour']
    prev_pos = np.array(self.joints[joint_name]['pos'])

    mask = cv2.inRange(image, colour[0], colour[1])
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=1)
    M = cv2.moments(mask)

    if M['m00'] == 0:
        # joint is hidden behind another joint. Return the last known position
        # and let the other script figure it out
        return np.array([time.time(), prev_pos[0], prev_pos[1]]) 
    cx = float(M['m10'] / M['m00'])
    cy = float(M['m01'] / M['m00'])

    self.joints[joint_name]['pos'] = np.array([cx, cy])
    return np.array([time.time(), cx, cy])

  
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

    j3_pos = self.detect_joint_pos(self.cv_image2, 'joint_3')
    j4_pos = self.detect_joint_pos(self.cv_image2, 'joint_4')
    ee_pos = self.detect_joint_pos(self.cv_image2, 'end_effector')
    j1_pos = self.detect_joint_pos(self.cv_image2, 'joint_1')

    a = pixel_to_meters(j1_pos[1], j3_pos[1], 2.5)
    j3_pos[1] *= a
    j4_pos[1] *= a
    ee_pos[1] *= a
    # Publish the results
    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.joints['joint_3']['pub'].publish(Float64MultiArray(LAYOUT, j3_pos))
      self.joints['joint_4']['pub'].publish(Float64MultiArray(LAYOUT, j4_pos))
      self.joints['end_effector']['pub'].publish(Float64MultiArray(LAYOUT, ee_pos))
    except CvBridgeError as e:
      print(e)

# call the class
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


