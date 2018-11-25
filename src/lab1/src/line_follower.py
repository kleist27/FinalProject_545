#!/usr/bin/env python

import collections
import sys
import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseArray, PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
import utils

# The topic to publish control commands to
PUB_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0' 
# The topic to subscribe to sim car pose
#SUB_TOPIC = '/sim_car_pose/pose' this is the pose_topic
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

    # Store the passed parameters
    self.plan = plan
    print(self.plan)
    self.plan_lookahead = plan_lookahead

    # Normalize translation and rotation weights
    self.translation_weight = translation_weight / (translation_weight+rotation_weight)
    self.rotation_weight = rotation_weight / (translation_weight+rotation_weight)
    self.kp = kp
    self.ki = ki
    self.kd = kd

    # The error buff stores the error_buff_length most recent errors and the
    # times at which they were received. That is, each element is of the form
    # [time_stamp (seconds), error]. For more info about the data struct itself, visit
    # https://docs.python.org/2/library/collections.html#collections.deque
    self.error_buff = collections.deque(maxlen=error_buff_length)
    self.error_buff.append((0, 0))
    self.speed = speed
    
    # Publisher
    self.cmd_pub = rospy.Publisher(PUB_TOPIC, AckermannDriveStamped, queue_size = 1) 
    # Subscriber
    self.pose_sub = rospy.Subscriber(pose_topic, PoseStamped, self.pose_cb)
  
  '''
  Computes the error based on the current pose of the car
    cur_pose: The current pose of the car, represented as a numpy array [x,y,theta]
  Returns: (False, 0.0) if the end of the plan has been reached. Otherwise, returns
           (True, E) - where E is the computed error
  '''
  def compute_error(self, cur_pose):
    minimum_gap = 0.15
    # Find the first element of the plan that is in front of the robot, and remove
    # any elements that are behind the robot. To do this:
    # Loop over the plan (starting at the beginning) For each configuration in the plan
        # If the configuration is behind the robot, remove it from the plan
        #   Will want to perform a coordinate transformation to determine if 
        #   the configuration is in front or behind the robot
        # If the configuration is in front of the robot, break out of the loop
    if len(self.plan) > 0:
      # YOUR CODE HERE
      cur_x = cur_pose[0]
      cur_y = cur_pose[1]
      cur_theta = cur_pose[2]
      goal_idx = min(0+self.plan_lookahead, len(self.plan)-1)
      next_wp_xpose=self.plan[goal_idx].position.x
      next_wp_ypose=self.plan[goal_idx].position.y
      next_wp_theta=utils.quaternion_to_angle(self.plan[goal_idx].orientation)
      # Compute the translation error between the robot and the configuration at goal_idx in the plan
      translation_error = math.sqrt(((next_wp_xpose-cur_x)**2)+((next_wp_ypose-cur_y)**2))
      # Compute the total error
      # Translation error was computed above
      # Rotation error is the difference in yaw between the robot and goal configuration
      #   Be careful about the sign of the rotation error
      rotation_error =  next_wp_theta - cur_theta # New Theta - Current Steering Angle
      error = self.translation_weight * translation_error + self.rotation_weight * abs(rotation_error)
      '''# Tyler Code --------------------
      plan_temp = self.plan[0]

      #Used to check plan length
      print(len(self.plan))

      #Angle difference between first plan point & pose
      angle_val = cur_theta - self.plan[0].orientation.w
      
      #Rotation Matrix for plan and pose
      rot_mat = utils.rotation_matrix(angle_val)
      
      #XY pose and plan
      cur_xy = np.matrix([[cur_pose[0]], [cur_pose[1]]])
      plan_xy = np.matrix([[plan_temp.position.x], [plan_temp.position.y]])

      #used to find the translation between pose and plan
      v1 = np.matmul(rot_mat,plan_xy)
      translation_mat = np.subtract(cur_xy,v1)

      #shows the difference between the points
      print(translation_mat) 

      #If the X translation is negative the plan is behind the pose
      if  translation_mat[0,0] < 0:
        self.plan = np.delete(self.plan, (0), axis=0) #deletes the first point in the plan
 
      '''# Tyler Code --------------------       
      for i in range(len(self.plan)):
        if translation_error < minimum_gap:
          self.plan=np.delete(self.plan, (0), axis=(0))
          break
    # Check if the plan is empty. If so, return (False, 0.0)
    # YOUR CODE HERE
    elif len(self.plan) == 0:
      return(False, 0.0)
    # At this point, we have removed configurations from the plan that are behind
    # the robot. Therefore, element 0 is the first configuration in the plan that is in 
    # front of the robot. To allow the robot to have some amount of 'look ahead',
    # we choose to have the robot head towards the configuration at index 0 + self.plan_lookahead
    # We call this index the goal_index
    
    if rotation_error < 0:
      error *= -1
    print(error)
    return True, error
    
    
  '''
  Uses a PID control policy to generate a steering angle from the passed error
    error: The current error
  Returns: The steering angle that should be executed
  '''
  def compute_steering_angle(self, error):
    now = rospy.Time.now().to_sec() # Get the current time
    # Compute the derivative error using the passed error, the current time,
    deriv_error = (error - (self.error_buff[0][0]))/(now - (self.error_buff[0][1])) # [0][0] for error buffer gets you the first point, first error val
    # Add error to buffer
    self.error_buff.append((error, now))
    '''
    for i in self.error_buff[i][0]:
      if i > 4: 
        break
      else:
        integ_err_num += self.error_buff[i]-self.error_buff[i+1]
    for i in self.error_buff[i][1] 
      if i + 1 == 6: 
        break
      else:
        integ_error_den += self.error_buff[i]-self.error_buff[i+1]      
    '''
    integ_error= float(((self.error_buff[0][0]-self.error_buff[1][0])+
                  (self.error_buff[1][0]-self.error_buff[2][0])+
                  (self.error_buff[2][0]-self.error_buff[3][0])+
                  (self.error_buff[3][0]-self.error_buff[4][0]))
                  )/float(now-(self.error_buff[4][1]))
    # Compute the steering angle as the sum of the pid errors
    # print([error,deriv_error])
    #print([error,integ_error,deriv_error])
    #print(integ_error)
    return self.kp*error + self.ki*integ_error + self.kd*deriv_error
    
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
      # We have reached our goal
      self.pose_sub = None # Kill the subscriber
      self.speed = 0.0 # Set speed to zero so car stops
      
    delta = self.compute_steering_angle(error)
    
    # Setup the control message
    ads = AckermannDriveStamped()
    ads.header.frame_id = '/map'
    ads.header.stamp = rospy.Time.now()
    ads.drive.steering_angle = delta
    ads.drive.speed = self.speed
    
    # Send the control message
    self.cmd_pub.publish(ads)

def main():

  # Initialize the node
  rospy.init_node('line_follower', anonymous=True) 

  #Grab Params from launch file
  plan_topic = rospy.get_param("~plan_topic")
  pose_topic = rospy.get_param("~pose_topic")
  plan_lookahead = rospy.get_param("~plan_lookahead")
  translation_weight = rospy.get_param("~translation_weight")
  rotation_weight = rospy.get_param("~rotation_weight")
  kp = rospy.get_param("~kp")
  ki = rospy.get_param("~ki")
  kd = rospy.get_param("~kd")
  error_buff_length = rospy.get_param("~error_buff_length")
  speed = rospy.get_param("~speed")
  
  #Grab Params from launch file
  #plan_topic = "/planner_node/car_plan"
  #print(plan_topic)
  #pose_topic = "/sim_car_pose/pose"
  #plan_lookahead = 5
  #translation_weight =  1.0
  #rotation_weight = 0.5 
  #kp = 1.0
  #ki = 0.0
  #kd = 0.0
  #error_buff_length = 10
  #speed = 1.0

  raw_input("Press Enter to when plan available...")  # Waits for ENTER key press
  
  # Use rospy.wait_for_message to get the plan msg
  plan = rospy.wait_for_message(plan_topic, PoseArray)
  '''
  plan_length = (np.matrix(plan.poses)).size  # (np.matrix(plan.poses)).size
  plan_array = []
  print(plan)
  # Convert the plan msg to a list of 3-element numpy arrays
  #     Each array is of the form [x,y,theta]
  for i in range(plan_length):
    plan_array.append(np.array([plan.poses[i].position.x,
                         plan.poses[i].position.y,
                         utils.quaternion_to_angle(plan.poses[i].orientation)]))
  '''
  # Create a LineFollower object
  lf = LineFollower(plan.poses, pose_topic, plan_lookahead, translation_weight, rotation_weight, kp, ki, kd, error_buff_length, speed)
  rospy.spin() # Prevents node from shutting down

if __name__ == '__main__':
  main()