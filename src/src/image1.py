#!/usr/bin/env python3

import json
import sys
import roslib
import rospy
import cv2
import move
from move import BLUE, RED, GREEN, YELLOW, ORANGE
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


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

    # need this to track target

    self.template = cv2.imread("sphere_tmplt.png", 0)
    self.target_pub = rospy.Publisher('target_est', String, queue_size=10)
    self.joints_pub = rospy.Publisher('joints_est', String, queue_size=10)
    # this will convert pixels to meters

    self.joints = {
            'joint_1': {
                'pos': np.array([0.0, 0.0, 0.0]),
                'colour': YELLOW
                },
            'joint_2': {
                'pos': np.array([0.0, 0.0, 0.0]),
                'colour': BLUE,
                # 'pub': rospy.Publisher('joint2_topic', Float64MultiArray, queue_size=10)
                },
            'joint_4': {
                'pos': np.array([0.0, 0.0, 0.0]),
                'colour': GREEN,
                # 'pub': rospy.Publisher('joint4_topic', Float64MultiArray, queue_size=10)
                },
            'end_effector': {
                'pos': np.array([0.0, 0.0, 0.0]),
                'colour': RED,
                # 'pub': rospy.Publisher('ee_topic', Float64MultiArray, queue_size=10)
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
        return np.array([prev_pos[0], 0.0, prev_pos[1]])
    cx = float(M['m10'] / M['m00'])
    cy = float(M['m01'] / M['m00'])

    self.joints[joint_name]['pos'] = np.array([cx, 0.0, cy])
    return np.array([cx, 0.0, cy])

  def detect_target(self, image):
    # isolate the orange objects
    mask = cv2.inRange(image, ORANGE[0], ORANGE[1])
    # remove everything else (all else goes black)
    res = cv2.bitwise_and(image, image, mask=mask)
    # convert to greyscale 
    grey = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    # match with template of orange sphere 
    # can we use this method? Is it cheating?
    # TODO follow example of Lab 2
    # accept fate if it is too late though
    res = cv2.matchTemplate(grey, self.template, cv2.TM_CCOEFF_NORMED)

    _, _, _, max_loc = cv2.minMaxLoc(res)

    return np.array([max_loc[0], 0.0, max_loc[1]])

  # Recieve data from camera 1, process it, and publish
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    # Uncomment if you want to save the image
    #cv2.imwrite('image_copy.png', cv_image)

    cv2.imshow('window1', self.cv_image1)
    cv2.waitKey(1)
    
    self.mover.move()
    timestamp = rospy.get_rostime()
    print(timestamp)
    j1_pos = self.detect_joint_pos(self.cv_image1, 'joint_1')
    j2_pos = self.detect_joint_pos(self.cv_image1, 'joint_2')
    j4_pos = self.detect_joint_pos(self.cv_image1, 'joint_4')
    ee_pos = self.detect_joint_pos(self.cv_image1, 'end_effector')
    target_pos = self.detect_target(self.cv_image1)

    a = pixel_to_meters(j1_pos, j2_pos, 2.5)
    j2_pos = a*(j1_pos - j2_pos)
    j4_pos = a*(j1_pos - j4_pos)
    ee_pos = a*(j1_pos - ee_pos)
    target_pos = a*(j1_pos-target_pos)
    # Publish the results


    # serialize data to json string to send over the streams
    joint_json = json.dumps({
        'projection': 'xz',
        'time': {
            'secs': timestamp.secs,
            'nsecs': timestamp.nsecs
            },
        'j1': j1_pos.tolist(),
        'j23': j2_pos.tolist(),
        'j4': j4_pos.tolist(),
        'ee': ee_pos.tolist()
        })

    target_json = json.dumps({
        'projection': 'xz',
        'time': {
            'secs': timestamp.secs,
            'nsecs': timestamp.nsecs
            },
        'pos': target_pos.tolist(),
        })
    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      self.target_pub.publish(String(target_json))
      self.joints_pub.publish(String(joint_json))
    except CvBridgeError as e:
      print(e)

def pixel_to_meters(j1, j2, length):
    return length/np.sqrt(np.sum((j1-j2)**2))

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


