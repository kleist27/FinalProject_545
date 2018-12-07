#!/usr/bin/env python
#12/2/18
import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
import utils # <----------- LOOK AT THESE FUNCTIONS ***************************

PUB_TOPIC = '/world_pose_array' # The topic that you should publish to
MAP_TOPIC = 'static_map' # The service topic that will provide the map
#POSE_TOPIC = '/sim_car_pose/pose'
POSE_TOPIC = "/pf/viz/inferred_pose"

class PointMap:

  def __init__(self):
    self.pub = rospy.Publisher(PUB_TOPIC, PoseArray, queue_size=1) 
    self.viz_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.viz_sub, queue_size = 1)# Create a subscriber to the current position of the car

    self.quats = [[0,0,-0.15,-0.99],[0,0,-0.06,.99],[0,0,-0.33,0.99],[0,0,-0.81,0.57],
		 [0,0,-.99,.05],[0,0,-.99,.05],[0,0,-0.99,-0.088],[0,0,0.985,0.17],[0,0,0.985,0.17],[0,0,0.985,0.17],[0,0,0.98,0.19],[0,0,0.83,0.54],
		 [0,0,-0.99,-0.12],[0,0,-0.91,0.41],[0,0,-0.86, 0.52],[0,0,-0.85,0.53],[0,0,-0.997,-0.07],[0,0,0.91,0.41],
		 [0,0,0.92,0.39],[0,0,0.99,0.001],[0,0,0.99,0.08],[0,0,0.99,0.08],
         	 [0,0,-0.98,0.188],[0,0,-0.97,0.24],[0,0,-0.97,0.23],[0,0,-0.97,0.21],[0,0,-0.81,0.57]]  

    self.poses = [[49.29,14.00],[50.29,14.34],[51.35,14.00],[52,11.85],
		 [50.97,10.65],[49.49,10.37],[47.33,10.2],[45.79,10.95],[43.29,12.00],[40.35,12.86],[39.00,13.26],
		 [38.29,14.6],[36.56,17.5],[35.55,17.41],[34.85,15.99],[33.53,13.99],[30.97,12.05],
		 [29.85,12.20],[28.6,13.7],[26.79,15.07],[25.3,15.12],[22.86,15.08],
       		 [20.41,15.24],[18.42,14.93],[15.74,13.69],[13.26,12.12],[11.5,10.54]] 

    #'''[48.5,10.35],''''''[0,0,0.-99,-0.12],''' #7th
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
         #print(plan_array[k])
         #Ignore below for line_follower


         #Setup the pose
         pose_out = Pose()
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

