#!/usr/bin/env python

import collections
import sys

import rospy
import numpy as np
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from ackermann_msgs.msg import AckermannDriveStamped

import utils

# The topic to publish control commands to
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'

'''
Follows a given plan using constant velocity and PID control of the steering angle
'''
class LineFollower:

  '''
  Initializes the line follower
    plan: A list of length T that represents the path that the robot should follow
          Each element of the list is a 3-element numpy array of the form [x,y,theta]
    pose_topic: The topic that provides the current pose of the robot as a PoseStamped msg
    plan_lookahead: If the robot is currently closest to the i-th pose in the plan,
                    then it should navigate towards the (i+plan_lookahead)-th pose in the plan
    translation_weight: How much the error in translation should be weighted in relation
                        to the error in rotation
    rotation_weight: How much the error in rotation should be weighted in relation
                     to the error in translation
    kp: The proportional PID parameter
    ki: The integral PID parameter
    kd: The derivative PID parameter
    error_buff_length: The length of the buffer that is storing past error values
    speed: The speed at which the robot should travel
  '''
  def __init__(self, plan, pose_topic, plan_lookahead, translation_weight,
               rotation_weight, kp, ki, kd, error_buff_length, speed):

    self.plan_lookahead = plan_lookahead
    self.translation_weight = translation_weight / (translation_weight+rotation_weight)
    self.rotation_weight = rotation_weight / (translation_weight+rotation_weight)
    self.kp = kp
    self.ki = ki
    self.kd = kd
    self.error_buff = collections.deque(maxlen=error_buff_length)
    self.plan = plan
    self.speed = speed
    
    self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size=1)
    
    self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb, queue_size=1)

  '''
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
  Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
  '''
  def compute_error(self, cur_pose):
    rot_mat = utils.rotation_matrix(-1*cur_pose[2])    
    while len(self.plan) > 0:
      # Figure out if self.plan[0] is in front or behind car
      offset = rot_mat * ((self.plan[0][0:2]-cur_pose[0:2]).reshape(2,1))
      offset.flatten() 
      if offset[0] > 0.0:
        break
        #print(offset)
      self.plan.pop(0)
     # print(self.plan)
   
    if len(self.plan) <= 0:
      return False, 0.0
   
    # Get the idx of point that we are heading towards
    goal_idx = min(self.plan_lookahead, len(self.plan)-1)
   
    # Compute the offset of goal point
    goal_offset = rot_mat * ((self.plan[goal_idx][0:2]-cur_pose[0:2]).reshape(2,1))
    goal_offset.flatten()
    # Compute error
    error = (self.translation_weight*goal_offset[1] + 
             self.rotation_weight * (self.plan[goal_idx][2]-cur_pose[2]))
    return True, error
    
  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''    
  def compute_steering_angle(self, error):
    now = rospy.Time.now().to_sec()
    
    derror_dt = 0.0
    if len(self.error_buff) >= 1:
      derror_dt = (error - self.error_buff[-1][0])/(now-self.error_buff[-1][1])
    
    self.error_buff.append((error, now))
    sum_error = 0.0
    for i in xrange(len(self.error_buff)-1):
      sum_error += 0.5*(self.error_buff[i][0]+self.error_buff[i+1][0])*(self.error_buff[i+1][1]-self.error_buff[i][1])
    
    return self.kp*error + self.ki*sum_error + self.kd * derror_dt

  '''
  Callback for the current pose of the car
    msg: A PoseStamped representing the current pose of the car
    This is the exact callback that we used in our solution, but feel free to change it
  '''      
  def pose_cb(self, msg):
    cur_pose = np.array([msg.pose.position.x,
                         msg.pose.position.y,
                         utils.quaternion_to_angle(msg.pose.orientation)])
    success, error = self.compute_error(cur_pose)
    
    if not success:
      self.pose_sub = None
      self.speed = 0.0
      
    delta = self.compute_steering_angle(error)
    
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed
    
    self.cmd_pub.publish(ads)
    
def pose_to_config(msg):
    return np.array([msg.position.x,
                     msg.position.y,
                     utils.quaternion_to_angle(msg.orientation)], 
                    np.float)  

'''def pose_array_to_plan(msg):
  result = []
  for i in xrange(len(msg.poses)):
    result.append(pose_to_config(msg.poses[i]))
  return result'''

def pose_array_to_plan(msg):
  result = []
  for i in xrange(len(msg.poses)):
    result.append(pose_to_config(msg.poses[i]))
  return result

def main():

  rospy.init_node('line_follower', anonymous=True) # Initialize the node
  
  plan_topic = rospy.get_param('~plan_topic', '/world_pose_array')
  pose_topic = rospy.get_param('~pose_topic', '/sim_car_pose/pose')
  plan_lookahead = rospy.get_param('plan_lookahead', 0)
  translation_weight = rospy.get_param('~translation_weight', 1.0)
  rotation_weight = rospy.get_param('~rotation_weight', 0.0)
  kp = rospy.get_param('~kp', 1.0)
  ki = rospy.get_param('~ki', 0.0)
  kd = rospy.get_param('~kd', 0.0)
  error_buff_length = rospy.get_param('~error_buff_length', 10)
  speed = rospy.get_param('~speed', 1.0)

  #quats = [[0,0,0.15,0.99],[0,0,-0.7,0.67],[0,0,0.998,0.05],[0,0,0.985,0.17],[0,0,0.98,0.19],[0,0,0.83,0.54],[0,0,-0.99,-0.12],[0,0,-0.9,0.42],	[0,0,-0.99,0.07],[0,0,0.91,0.41],[0,0,0.92,0.39],[0,0,0.99,0.001],[0,0,0.99,0.08], [0,0,-0.99,0.11],[0,0,-0.97,0.21],[0,0,-0.81,0.57]]  

  #poses = [[49.29,12.4],[52,11.75],[48.21,10.21],[43.79,11.51],[39.25,13.26],[38.29,14.6],[36.56,17.5],[32.9,14.35],[31.1,11.17],        [29.85,12.20],[28.6,13.7],[26.79,15.07],[25.3,15.12],[20.77,15.83],[13.26,12.12],[11.5,10.54]]
  #quats = [[0,0,-0.987,-0.158],[0,0,-0.987,-0.158],[0,0,-0.987,-0.158]]
  #poses = [[48.99,14.79],[45.05,17.75],[40.05,17.71]]
  #plan_array = []          
                 
  #for k in xrange(len(quats)):
   #    temp_xy = poses[k]
    #   temp_q = quats[k]
     #  ori = Pose()
      # ori.orientation.x = temp_q[0]
       #ori.orientation.y = temp_q[1]
       #ori.orientation.z = temp_q[2]
       #ori.orientation.w = temp_q[3]
       #plan_array.append(np.array([temp_xy[0], temp_xy[1], utils.quaternion_to_angle(ori.orientation)]))
       #print(plan_array[k])

  #plan = plan_array
  #print(plan)
  while not rospy.is_shutdown():   
    raw_input("Press Enter to when plan available...")  
    pa_plan = rospy.wait_for_message(plan_topic, PoseArray)
    plan = pose_array_to_plan(pa_plan)  
    plan_end = plan[-1]  
    lf = LineFollower(plan, pose_topic, plan_lookahead, translation_weight,
             rotation_weight, kp, ki, kd, error_buff_length, speed)
    while lf.pose_sub is not None:
      rospy.sleep(1.0)
    print 'Reached the goal'

if __name__ == '__main__':
  main()
