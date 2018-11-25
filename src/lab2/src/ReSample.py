#!/usr/bin/env python

import rospy
import numpy as np
from threading import Lock
np.set_printoptions(threshold='nan')
'''
  Provides methods for re-sampling from a distribution represented by weighted samples
'''
class ReSampler:

  '''
    Initializes the resampler
    particles: The particles to sample from
    weights: The weights of each particle
    state_lock: Controls access to particles and weights
  '''
  def __init__(self, particles, weights, state_lock=None):
    self.particles = particles 
    self.weights = weights
    self.m_inv = (1.0/(len(self.particles)))
    self.temp_array = temp_array = np.zeros(self.particles.shape)

    if state_lock is None:
      self.state_lock = Lock()
    else:
      self.state_lock = state_lock
  
  '''
    Performs independently, identically distributed in-place sampling of particles
  '''
  def resample_naiive(self):
    self.state_lock.acquire()  
   
    #creates an array that is arranged with the length of particles 
    idx_array = np.arange(len(self.particles)) 

    #applies the weights and returns the index of the particles to be stored
    array_idx = np.random.choice(idx_array.tolist(), len(self.particles), p=self.weights.tolist())

    #Stores all of the values for rewriting back to self.particles
    for i in range(len(self.particles)):
      self.temp_array[i][:] = self.particles[(array_idx[i])]

    #Copies back the sampled numbers
    self.particles[:] = self.temp_array[:]

    self.state_lock.release()
  
  '''
    Performs in-place, lower variance sampling of particles
    (As discussed on pg 110 of Probabilistic Robotics)
  '''
  def resample_low_variance(self):
    self.state_lock.acquire()

    #calculates the random value
    r = self.m_inv * np.random.random_sample((1, 1))

    #stores initial weight value
    c = self.weights[0]

    #Used for indexing
    i = 0
    
    #Loops and stores the values into temp array
    for k in range(len(self.particles)):
      U = r + (k)*self.m_inv
      while U > c:
        i += 1
        c += self.weights[i]
      self.temp_array[k][:] = self.particles[i][:]

    #Writes back the values in the temp array
    self.particles[:] = self.temp_array[:]

    self.state_lock.release()
    
import matplotlib.pyplot as plt

if __name__ == '__main__':

  rospy.init_node("sensor_model", anonymous=True) # Initialize the node

  n_particles = int(rospy.get_param("~n_particles",100)) # The number of particles    
  k_val = int(rospy.get_param("~k_val", 80)) # Number of particles that have non-zero weight
  resample_type = rospy.get_param("~resample_type", "low_variance") # Whether to use naiive or low variance sampling
  trials = int(rospy.get_param("~trials", 10)) # The number of re-samplings to do
  
  histogram = np.zeros(n_particles, dtype=np.float) # Keeps track of how many times
                                                    # each particle has been sampled
                                                    # across trials
  for i in xrange(trials):
    particles = np.repeat(np.arange(n_particles)[:,np.newaxis],3, axis=1) # Create a set of particles
                                                                          # Here their value encodes their index
    # Have increasing weights up until index k_val
    weights = np.arange(n_particles, dtype=np.float)
    weights[k_val:] = 0.0
    weights[:] = weights[:] / np.sum(weights)
    
    rs = ReSampler(particles, weights) # Create the Resampler
    #print(particles)
    # Resample
    if resample_type == "naiive":
      rs.resample_naiive()
    elif resample_type == "low_variance":
      rs.resample_low_variance()
    else:
      print "Unrecognized resampling method: "+ resample_type     

    # Add the number times each particle was sampled    
    for j in xrange(particles.shape[0]):
      histogram[particles[j,0]] = histogram[particles[j,0]] + 1

  # Display as histogram
  plt.bar(np.arange(n_particles), histogram)
  plt.xlabel('Particle Idx')
  plt.ylabel('# Of Times Sampled')
  plt.show()    