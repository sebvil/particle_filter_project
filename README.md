# particle_filter_project

## Team members: Hunter Thompson, Sebastian Villegas Mejia


## Implementation Plan

### Initialize Particle Cloud

From the filter's `map` attribute, we first obtain the indexes of the 0 values
and map them to coordinates. We draw a simple random sample with replacement
of size $n$ (TBD). Then, for each particle, we randomly select its orientation.

To test this, we will create a visualization of the particles on the map similar
to the one on the project website.

### Update particle position

For each particle, we update displace it or rotate it based on the sensed 
movement from the robot. 

To test this, we will create a test function where the particles and their
movements will be hardcoded and checked against their updated positions are
within some range of the expected position.

### Compute importance weights

A vector for the expected recordings of each particle is calculated based on 
the map data and the pose of the particle. Then, the weight is calculated from
the inverse of the distance between the expected recording and the actual 
recording.

To test this, we will create a test function where the particles are hardcoded
and the calculated weights will be checked against expected weights.


### Normalize weights

The sum of all the weights will be calculated, and then each weight will get 
divided by this sum. Then, the function `draw_random_sample` will be used to
resample based on these calculated probabilities.

To test this, we will create a test function where the particles are hardcoded
and the calculated normalizes weights will be checked against expected 
normalized weights.

### Estimate robot's pose

The mean of the new sample of particles is calculated. This mean is the estimated
pose.

To test this, we will create a test function where the particles are hardcoded
and the calculated estimated pose is compared against the expected pose.

### Noise incorporation

Whenever a particle is moved, some Gaussian noise is added to each particle's 
movement.

When testing the updating of the particles position, it will be checked that 
all the particles do not have values exactly equal to the values that would
occur without noise.

### Timeline

`initialize_particle_cloud` and `update_particles_with_motion_model` — by 
Monday Feb 1st. 

`update_particle_weights_with_measurement_model` — by 
Thursday Feb 4th. 

`normalize_particles`, `resample_particles`, and `update_estimated_robot_pose`
— by Friday Feb 5th.


# Writeup

## Project Objectives

The objectives of this project were to gain experience and knowledge with the
problem of robot localization. The goal was to use a probabilistic approach
to solve this problem, specifically, the particle filter algorithm.

## High Level Description


## Particle Filter Algorithm

### Movement

The movement of the particles is handled by the functions `robot_scan_received`
and `update_particles_with_motion_model`.

- `robot_scan_received`: first, this function checks to make sure that some 
  conditions pass, such as the particle cloud being initialized, having a 
  previous odometer pose, and having displaced enough to update the particles. 
  If these conditions pass, the function makes a call to 
  `update_particles_with_motion_model`, which updates the pose of each particle.
- `update_particles_with_motion_model`: for all the particles in the particle 
  cloud, this function will update the x and y positions and the yaw of each 
  particle. The change in each direction is drawn from a $N(n, 0.1^2n^2)$ 
  distribution, where $n$ is the difference in the odometry readings for the 
  desired direction.

### Computation of importance weights

- `normalize_particles`: this function divided the weight of each particle by 
  the sum of the weights of all particles, effectively making the sum of all 
  weights be 1.

### Resampling


## Challenges

## Future work


## Takeaways

- The implementation plan was definitely very useful for our group. It helped us
  gain a basic understanding of the tasks we were supposed to do and the 
  particle filter algorithm before we began working on it. Additionally, it gave
  us the opportunity to split up the work fairly ahead of time, and by
  setting deadlines, we were able to have a strong basis for the project days
  before the due date.


# Demo
![Particle filter demo](particle_filter.gif)
