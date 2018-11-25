#!/usr/bin/env python
import rospy
import numpy as np
import math
import sys

import utils

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseArray, Pose

SCAN_TOPIC = '/scan' # The topic to subscribe to for laser scans
CMD_TOPIC = '/vesc/high_level/ackermann_cmd_mux/input/nav_0' # The topic to publish controls to
POSE_TOPIC = '/sim_car_pose/pose' # The topic to subscribe to for current pose of the car
                                  # NOTE THAT THIS IS ONLY NECESSARY FOR VIZUALIZATION
VIZ_TOPIC = '/laser_wanderer/rollouts' # The topic to publish to for vizualizing
                                       # the computed rollouts. Publish a PoseArray.

MAX_PENALTY = 10000 # The penalty to apply when a configuration in a rollout
                    # goes beyond the corresponding laser scan
                    

'''
Wanders around using minimum (steering angle) control effort while avoiding crashing
based off of laser scans. 
'''
class LaserWanderer:

  '''
  Initializes the LaserWanderer
    rollouts: An NxTx3 numpy array that contains N rolled out trajectories, each
              containing T poses. For each trajectory, the t-th element represents
              the [x,y,theta] pose of the car at time t+1
    deltas: An N dimensional array containing the possible steering angles. The n-th
            element of this array is the steering angle that would result in the 
            n-th trajectory in rollouts
    speed: The speed at which the car should travel
    compute_time: The amount of time (in seconds) we can spend computing the cost
    laser_offset: How much to shorten the laser measurements
  '''
  def __init__(self, rollouts, deltas, speed, compute_time, laser_offset, car_angle, car_length):
    # Store the params for later
    self.rollouts = rollouts
    self.deltas = deltas
    self.speed = speed
    self.compute_time = compute_time
    self.laser_offset = laser_offset
    self.car_angle = car_angle
    self.car_length = car_length
    
    # YOUR CODE HERE
    self.cmd_pub = rospy.Publisher(CMD_TOPIC, AckermannDriveStamped, queue_size = 1) # Create a publisher for sending controls
    self.laser_sub = rospy.Subscriber(SCAN_TOPIC, LaserScan, self.wander_cb, queue_size = 1)# Create a subscriber to laser scans that uses the self.wander_cb callback
    self.viz_pub = rospy.Publisher(VIZ_TOPIC, PoseArray, queue_size = 1) # Create a publisher for vizualizing trajectories. Will publish PoseArrays
    self.viz_sub = rospy.Subscriber(POSE_TOPIC, PoseStamped, self.viz_sub_cb, queue_size = 1)# Create a subscriber to the current position of the car
    # NOTE THAT THIS VIZUALIZATION WILL ONLY WORK IN SIMULATION. Why?
    
  '''
  Vizualize the rollouts. Transforms the rollouts to be in the frame of the world.
  Only display the last pose of each rollout to prevent lagginess
    msg: A PoseStamped representing the current pose of the car
  '''  
  def viz_sub_cb(self, msg):

    # Create the PoseArray to publish
    pa = PoseArray()
    pa.header.frame_id = '/map'
    pa.header.stamp = rospy.Time.now()
    
    for k in range(len(self.rollouts)):
      
      #initializing the new pose to append to the array
      pose_out = Pose()

      #find out length of trajectories for indexing to the end
      traj_length = len(self.rollouts[1,:])-1

      #Angle difference between first plan point & pose
      rotation_angle = self.rollouts[k,traj_length,2] + utils.quaternion_to_angle(msg.pose.orientation)

      #Rotation Matrix for pose of car
      rot_mat = utils.rotation_matrix(rotation_angle)

      #XY of current pose vector
      cur_xy = np.matrix([[msg.pose.position.x], [msg.pose.position.y]])


      #XY postion of the last rollout
      rollout_xy = ([[self.rollouts[k,traj_length,0]], [self.rollouts[k,traj_length,1]]])

      #Rotation matrix multiplication
      v1 = np.matmul(rot_mat,rollout_xy)
      
      #Add the transformation
      v_out = np.add(v1, cur_xy)

      #Setup the pose
      pose_out.position.x = v_out[0,0]
      pose_out.position.y = v_out[1,0]
      pose_out.position.z = 0
      pose_out.orientation = utils.angle_to_quaternion(rotation_angle)

      #Append the pose onto the array
      pa.poses.append(pose_out)

    pass

    #publish the PoseArray to the screen
    self.viz_pub.publish(pa)
    
    
  '''
  Compute the cost of one step in the trajectory. It should penalize the magnitude
  of the steering angle. It should also heavily penalize crashing into an object
  (as determined by the laser scans)
    delta: The steering angle that corresponds to this trajectory
    rollout_pose: The pose in the trajectory 
    laser_msg: The most recent laser scan
  '''  
  def compute_cost(self, delta, rollout_pose, laser_msg):
  
    # Initialize the cost to be the magnitude of delta
    cost = np.absolute(delta)
    #print(delta)
    # distance from the car to the rollout_pose
    rollout_distance = np.sqrt(np.square(rollout_pose[0])+np.square(rollout_pose[1]))

    #angle from the cars x axis to rollout point
    rollout_angle = np.arcsin(rollout_pose[1]/rollout_distance) 

    # Find the laser ray that corresponds to this angle & read it
    laser_range = 681/(np.absolute(laser_msg.angle_min)+np.absolute(laser_msg.angle_max))
    laser_index = int((rollout_angle+laser_msg.angle_max)*laser_range)
    laser_read = laser_msg.ranges[laser_index]
    
    # Checks for bad laser reads, then adds MAX_PENALTY to the cost if the distance from the robot to the rollout_pose is greater than the laser distance
    if ((np.isnan(laser_read)) == True) or (laser_read == 0) or (rollout_distance <= laser_read):
      cost += 0
    else:
      cost += MAX_PENALTY 

    #Checks the scans at the width of of the real car to make sure it won't swipe objects
    if (np.absolute(delta) < 0.05):
      rollout_distance = self.car_length + (self.car_length*2) * np.cos(self.car_angle)
      laser_car_index1 = int((rollout_angle+laser_msg.angle_max+self.car_angle)*laser_range)
      laser_car_index2 = int((rollout_angle+laser_msg.angle_max-self.car_angle)*laser_range)
      laser_read1 = laser_msg.ranges[laser_car_index1]
      laser_read2 = laser_msg.ranges[laser_car_index2]
      if (rollout_distance >= laser_read1) or (rollout_distance >= laser_read2):
        cost += MAX_PENALTY
   
    return cost
  '''
  Controls the steering angle in response to the received laser scan. Uses approximately
  self.compute_time amount of time to compute the control
    msg: A LaserScan
  '''
  def wander_cb(self, msg):

    # Get the time at which this function started
    start = rospy.Time.now().to_sec()

    #variable initialization
    delta_costs = np.zeros(self.deltas.shape[0], dtype=np.float) 
    traj_depth = 0

    #adds the cost up until the time limit has been met or the furthest trajectory cost has been calculated
    while((rospy.Time.now().to_sec() < start + self.compute_time) and (traj_depth < (len(self.rollouts[1,:])-1))):
       for n in range(len(delta_costs)):
         delta_costs[n] += self.compute_cost(self.deltas[n], self.rollouts[n,traj_depth], msg) # cost of the t=traj_depth step of trajectory n
       traj_depth += 1 
       print(traj_depth)


    # Find the delta that has the smallest cost
    min_cost = delta_costs.argmin(0) 

    #prep the drive commands to publish
    move = AckermannDriveStamped()
    move.header.frame_id = '/map'
    move.header.stamp = rospy.Time.now()
    move.drive.speed = self.speed
    move.drive.steering_angle = (self.deltas[min_cost]*-1)

    #publish the commands with the best path
    self.cmd_pub.publish(move)
    
'''
Apply the kinematic model to the passed pose and control
  pose: The current state of the robot [x, y, theta]
  control: The controls to be applied [v, delta, dt]
  car_length: The length of the car
Returns the resulting pose of the robot
'''
def kinematic_model_step(pose, control, car_length):
  
  # Consider the case where delta == 0.0, can't divide by 0, made it a very small value
  if control[1] == 0:
    control[1] = .0001

  #sin(2*Beta) value as it is used in each equation
  sin_2beta = np.sin(2*(np.arctan((.5 * np.tan(control[1])))))

  #next theta value
  theta_t = pose[2] + ((control[0] / car_length) * sin_2beta * control[2])
  
  #next X value
  x_t = pose[0] + (car_length/sin_2beta)*(np.sin(theta_t)-np.sin(pose[2])) 

  #next Y value
  y_t = pose[1] + (car_length/sin_2beta)*(np.cos(theta_t)-np.cos(pose[2]))

  # Makes sure your resulting theta is between 0 and 2*pi
  while (theta_t >= (2 * np.pi)) or (theta_t <= 0):
   if theta_t > 2 * np.pi:
     theta_t += -(2*np.pi)
   else:
     theta_t += (2*np.pi)

  #setup the return array
  next_rollout = [x_t, y_t, theta_t]

  return next_rollout
    
'''
Repeatedly apply the kinematic model to produce a trajectory for the car
  init_pose: The initial pose of the robot [x,y,theta]
  controls: A Tx3 numpy matrix where each row is of the form [v,delta,dt]
  car_length: The length of the car
Returns a Tx3 matrix where the t-th row corresponds to the robot's pose at time t+1
'''
def generate_rollout(init_pose, controls, car_length):

  #Variable setup
  cur_pose = init_pose
  next_poses = np.zeros((len(controls),3), dtype=np.float)

  #Loops through to calculate the X,Y, and theta for each rollout
  for i in range(len(controls)):
    next_poses[i] = kinematic_model_step(cur_pose, controls[i], car_length)
    cur_pose = next_poses[i] 
  return next_poses
   
'''
Helper function to generate a number of kinematic car rollouts
    speed: The speed at which the car should travel
    min_delta: The minimum allowed steering angle (radians)
    max_delta: The maximum allowed steering angle (radians)
    delta_incr: The difference (in radians) between subsequent possible steering angles
    dt: The amount of time to apply a control for
    T: The number of time steps to rollout for
    car_length: The length of the car
Returns a NxTx3 numpy array that contains N rolled out trajectories, each
containing T poses. For each trajectory, the t-th element represents the [x,y,theta]
pose of the car at time t+1
'''
def generate_mpc_rollouts(speed, min_delta, max_delta, delta_incr, dt, T, car_length):

  deltas = np.arange(min_delta, max_delta, delta_incr)
  N = deltas.shape[0]
  init_pose = np.array([0.0,0.0,0.0], dtype=np.float)
  
  rollouts = np.zeros((N,T,3), dtype=np.float)
  for i in xrange(N):
    controls = np.zeros((T,3), dtype=np.float)
    controls[:,0] = speed
    controls[:,1] = deltas[i]
    controls[:,2] = dt
    rollouts[i,:,:] = generate_rollout(init_pose, controls, car_length)
  return rollouts, deltas
 

def main():

  #Node Initialization
  rospy.init_node('laser_wanderer', anonymous=True)

  #Parameter initialization
  speed = rospy.get_param("~speed") # Default val: 1.0
  min_delta = rospy.get_param("~min_delta")# Default val: -0.34
  max_delta = rospy.get_param("~max_delta")# Default val: 0.341
  delta_incr = rospy.get_param("~delta_incr")# Starting val: 0.34/3 (consider changing the denominator) 
  dt = rospy.get_param("~dt")# Default val: 0.01
  T = rospy.get_param("~T")# Starting val: 300
  compute_time = rospy.get_param("~compute_time")# Default val: 0.09
  laser_offset = rospy.get_param("~laser_offset")# Starting val: 1.0
  
  #Car_length is already provided by teleop.launch
  car_length = rospy.get_param("car_kinematics/car_length", 0.33) 
  car_width = rospy.get_param("car_kinematics/car_length", 0.25)
  car_angle = np.arctan((car_length/2)/(car_width/2))

  # Generate the rollouts
  rollouts, deltas = generate_mpc_rollouts(speed, min_delta, max_delta,
                                           delta_incr, dt, T, car_length)
  
  # Create the LaserWanderer                                         
  lw = LaserWanderer(rollouts, deltas, speed, compute_time, laser_offset, car_angle, car_length)
  
  # Keep the node alive
  rospy.spin()
  

if __name__ == '__main__':
  main()