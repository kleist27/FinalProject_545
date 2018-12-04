#!/usr/bin/env python

import rospy
import numpy as np
import cv2 #as cv
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ackermann_msgs.msg import AckermannDriveStamped

SUB_TOPIC = '/camera/color/image_raw' 
PUB_TOPIC = '/camera/color/image_center'
CMD_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0' # The topic to publish controls to 

class VisionFinder:


  def __init__(self):

    #Used to swap images between ros and opencv
    self.bridge = CvBridge()

    #Publisher for DEBUG CV
    self.pub = rospy.Publisher(PUB_TOPIC, Image, queue_size=1)

    #Publisher for the movement commands
    self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size = 1) # Create a publisher for sending controls

    # Setup subscriber that subscribes that calls image_callback
    self.image_sub = rospy.Subscriber(SUB_TOPIC, Image, self.image_callback)

  def image_callback(self, msg):

    #Bridges image to CV or ouputs error
    try:
      in_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
      print(e)

    #Changes to HSV colorspace
    hsv = cv2.cvtColor(in_image, cv2.COLOR_BGR2HSV)

    #Limits on blue in HSV
    lower_blue = np.array([110,50,50])
    upper_blue = np.array([130,255,255])

    #Limits on red in HSV
    lower_red = np.array([5, 50, 50])
    upper_red = np.array([25, 255, 255])

    #masks for each color
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)


    #Can be used to combine color masks
    #mask = cv2.bitwise_or(mask1, mask2)
   
    #masks the image for desired color range
    output_blue = cv2.bitwise_and(in_image, in_image, mask = mask_blue)
    #output_red = cv2.bitwise_and(in_image, in_image, mask = mask_red)

##############################################################################

    #Setup array of masked image
    cvImg = cv2.cvtColor(output_blue, 6) #cv2.COLOR_BGR2GRAY
    npImg = np.asarray( cvImg )

    coordList = np.argwhere( npImg > 0 )
    numWhitePoints = len( coordList )

    #Checks if there is enough color in the image 
    if numWhitePoints > 100: #lower limit
      X=0;Y=0
      for (x,y) in coordList:
        X+=x
        Y+=y

      height = np.size(cvImg, 0)
      width = np.size(cvImg, 1)

      X_C = int(X/numWhitePoints)
      Y_C = int(Y/numWhitePoints)

      X_center=Y_C;Y_center=X_C #fix axes

      #DEBUG# Write the image with a circle in the center of the color.
      #DEBUG# 
      print("Center point: "+str(X_center)+","+str(Y_center))
      #DEBUG# 
      #Height 480:::::::::::: Width 640
      cv2.circle(in_image,(X_center,Y_center), 10, (0,255,0), -1)

      steer = ((X_center / 1600.0) -.20) * -1.0
      print(steer)

      move = AckermannDriveStamped()
      move.header.frame_id = '/map'
      move.header.stamp = rospy.Time.now()
      move.drive.speed = 0.65
      move.drive.steering_angle = steer

      #publish the commands with the best path
      self.cmd_pub.publish(move) 
      
    else:
      print("Not enough sample color")
      color_location = None

    #DEBUG# Publish the resulting image
    try:
      self.pub.publish(self.bridge.cv2_to_imgmsg(in_image, encoding="passthrough"))
    except CvBridgeError as e:
      print(e)

if __name__ == '__main__':

  sub_topic = rospy.get_param('~sub_topic', None)

  rospy.init_node('VisionFinder', anonymous=True) # Initialize the node
  
  vf = VisionFinder() # Create a clone follower

  rospy.spin() # Spin

