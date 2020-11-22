#!/usr/bin/env python3

import time
import sys
import roslib
import rospy
import cv2
import move
from move import BLUE, RED, GREEN
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError

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

class image_converter:

  # Defines publisher and subscriber
  def __init__(self):
    # initialize the node named image_processing
    rospy.init_node('image_processing', anonymous=True)
    # initialize a publisher to send images from camera1 to a topic named image_topic1
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    # initialize a subscriber to recieve messages rom a topic named /robot/camera1/image_raw and use callback function to recieve data
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    # initialize the bridge between openCV and ROS
    self.bridge = CvBridge()

    # need this to move the joints 
    self.mover = move.Mover()

    self.joints = {
            'joint_2': {
                'pos': np.array([0.0, 0.0, 0.0]),
                'colour': BLUE,
                'pub': rospy.Publisher('joint2_topic', Float64MultiArray, queue_size=10)
                },
            'joint_4': {
                'pos': np.array([0.0, 0.0, 0.0]),
                'colour': GREEN,
                'pub': rospy.Publisher('joint4_topic', Float64MultiArray, queue_size=10)
                },
            'end_effector': {
                'pos': np.array([0.0, 0.0, 0.0]),
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
    mask = cv2.dilate(mask, kernel, iterations=3)
    M = cv2.moments(mask)

    if M['m00'] == 0:
        # joint is hidden behind another joint. Return the last known position
        # and let the other script figure it out
        return prev_pos
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])

    self.joints[joint_name]['pos'] = np.array([cx, cy])
    return np.array([time.time(), cx, cy])

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    im1=cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    j2_pos = self.detect_joint_pos(self.cv_image1, 'joint_2')
    j4_pos = self.detect_joint_pos(self.cv_image1, 'joint_4')
    ee_pos = self.detect_joint_pos(self.cv_image1, 'end_effector')
    self.mover.move()

    # Publish the results
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.joints['joint_2']['pub'].publish(Float64MultiArray(LAYOUT, j2_pos))
      self.joints['joint_4']['pub'].publish(Float64MultiArray(LAYOUT, j4_pos))
      self.joints['end_effector']['pub'].publish(Float64MultiArray(LAYOUT, ee_pos))
    except CvBridgeError as e:
      print(e)

# call the class
def main(args):
  ic = image_converter()
  # joint2_pub = rospy.Publisher('/robot/joint2_position_controller/command', std_msgs.msg.Float64, queue_size=10)
  try:
      rospy.spin()
      print("hello")
      # for i in range(10):
          # a = (np.pi/2)*(np.sin((np.pi/15)*i))
          # joint2_pub.publish(std_msgs.msg.Float64(a))
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)


