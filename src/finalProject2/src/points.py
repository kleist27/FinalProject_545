#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import utils # <----------- LOOK AT THESE FUNCTIONS ***************************

PUB_TOPIC = '/world_pose_array' # The topic that you should publish to
MAP_TOPIC = 'static_map' # The service topic that will provide the map
POSE_TOPIC = '/sim_car_pose/pose'

class PointMap:

  def __init__(self):
    self.x = 0
    self.pub = rospy.Publisher(PUB_TOPIC, PoseArray, queue_size=1) 
    self.viz_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.viz_sub, queue_size = 1)# Create a subscriber to the current position of the car

    self.quats = [[0,0,0.15,0.99],[0,0,-0.33,0.99],[0,0,-0.39,0.91],[0,0,-0.7,0.67],[0,0,-0.86,0.51],   #2600,600
        [0,0,0.998,0.05],[0,0,0.985,0.17],[0,0,0.98,0.19],[0,0,0.83,0.54],[0,0,-0.99,-0.12],
	[0,0,-0.9,0.42],[0,0,-0.99,0.07],[0,0,0.91,0.41],[0,0,0.92,0.39],  #1435,545,
	[0,0,0.99,0.001],[0,0,0.99,0.08],  #1250,460
        [0,0,-0.99,0.11],[0,0,-0.97,0.21],[0,0,-0.85,0.51],[0,0,-0.57,0.82]]  #???

    self.poses = [[49.29,12.4],[51.35,12.60],[51.8,12.34],[52,11.75],[51.97,10.95],  #2600,660
        [48.21,10.21],[43.79,11.51],[39.25,13.26],[38.29,14.6],[36.56,17.5],
	[32.9,14.35],[31.1,11.17],[29.85,12.20],[28.6,13.7], #1435,545
	[26.79,15.07],[25.3,15.12], #1250,460
        [20.77,15.83],[13.26,12.12],[12.65,10.3],[13.07,8.28]]  #540,835


  def viz_sub(self, msg):
    pa = PoseArray()
    pa.header.frame_id = '/map'
    pa.header.stamp = rospy.Time.now()

    map_img, self.map_info = utils.get_map(MAP_TOPIC) # Get and store the map
    plan_array = []     
    map_pose = []      
                 
    for k in xrange(len(self.quats)):
         temp_xy = self.poses[k]
         temp_q = self.quats[k]
         ori = Pose()
         ori.orientation.x = temp_q[0]
         ori.orientation.y = temp_q[1]
         ori.orientation.z = temp_q[2]
         ori.orientation.w = temp_q[3]
         plan_array.append(np.array([temp_xy[0], temp_xy[1], utils.quaternion_to_angle(ori.orientation)]))


         pose_out = Pose()
         #Setup the pose
         pose_out.position.x = temp_xy[0]
         pose_out.position.y = temp_xy[1]
         pose_out.position.z = 0
         pose_out.orientation = ori.orientation

         #Append the pose onto the array
         pa.poses.append(pose_out)

         self.pub.publish(pa)
   
if __name__ == '__main__':
  
  rospy.init_node('point_map', anonymous=True) # Initialize the node
  
  pm = PointMap() # Create a clone follower
  rospy.spin() # Spin

