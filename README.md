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

The three main components of the approach were movement, weight computation,
and resampling of particles. These happened iteratively throughout the process
as the location of the robot was refined. First, the particles were initialized
randomly across the open spaces. Then, the movement step involved the bot
moving some amount, and each particle's movement was updated accordingly.
The weight computation step first involved computing the weights of the 
particles according to the likelihood field model, and then normalizing these
weights. Finally, the particles were randomly sampled with probability according
to their weights. From this point, we could estimate the bot's position by 
averaging the Pose() of each of the particles.

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

The computation of importance weights is initially handled by 
`update_particle_weights_with_measurement_model`, which computes the raw 
weights. Then, `normalize_particles` computes the actual weights after
normalization.

- `normalize_particles`: this function divided the weight of each particle by 
  the sum of the weights of all particles, effectively making the sum of all 
  weights be 1.
- `update_particle_weights_with_measurement_model`: This function takes in all 
  of the particles' position data and the robot's
  LaserScan data and finds the probability that the robot is colocalized with 
  each particle according to the likelihood field model. A high-level description
  of the code is that it goes through each particle's pose and from that 
  interprets how far off the robot's scan data is from the particle's predicted
  scan data using the likelhood_field that we were given from class. After 
  computing this across 4 angles, the particle's weight is updated accordingly
  (ie data that is closer has higher associated weight).

### Resampling

The resampling step is handled by the functions `resample_particles`, and then,
the position of the robot is estimated with `update_estimated_robot_pose`.

- `resample_particles`: this function first makes a list of the weights from
  the particle cloud. Then, it randomly samples the particles according to those
  weights using `draw_random_sample`.
- `update_estimated_robot_pose`: this function takes calculates the mean of the
  pose of all the particles and returns that as the robot's estimated position.
  This makes sense because although initially the estimated pose will be wildly
  wrong when particles are scattered across the map. Once most of the particles
  colocalize with the robot, the noise hopefully distributes the likelihood in
  such a way that the mean of the particles is a good estimate of bot pose.

## Challenges

The first biggest bug/challenge we encountered was that the robot's weights
did not add up to 1, so the resample function was not working. To solve this
issue, we found a bug in `update_particle_weights_with_measurement_model`. We
realized that the function `get_closest_obstacle_distance` returned nan when
the location was outside of the map and we decicded to continue looping in this 
edge case. Another challenge we ran into was balancing the computational 
efficiency of the number of particles and angles to check against the accuracy.
We found that using only 100 particles, while quite efficient rarely sampled
the open space thoroughly enough for the particles to actually colocalize with
the robot. Thus, we experimented around with values and found 10000 particles
to make the program run quite choppily on our hardware, but 5000 particles was
a good middle ground between the two. Similarly, we found that checking 8 
angles made the program run somewhat choppily with no visible improvement to
the algorithm. Thus, we chose to only check 4 directions (0, 90, 180, 270).
This checking was done both through visual inspection as well as printing
the processing time of the algorithm, which we tried to keep under one second
per iteration as a rule of thumb.

## Future work

One of the first things that comes to mind in terms of future work is making the
algorithm a bit more precise and computationally tractable. A possible means of
doing so may be reducing the number of particles in the cloud as they begin to
cluster around the bot location, while simultaneously increasing the number of
angles that are being checked. This might help more precisely locate the bot's
position keeping the same number of particles once the bot has more or less 
been localized may be somewhat redundant. Another means of improving the
algorithm with more time would be to compute the likelihood field for nan
values as well. When `get_closest_obstacle_distance` returns a nan value, the
the position could just barely be outside of the wall, much further, or 
anywhere in between. Thus, by including the values of closest distance for
these positions the particle filter might more easily discriminate between
particles and begin to cluster faster. 

## Takeaways

- The implementation plan was definitely very useful for our group. It helped us
  gain a basic understanding of the tasks we were supposed to do and the 
  particle filter algorithm before we began working on it. Additionally, it gave
  us the opportunity to split up the work fairly ahead of time, and by
  setting deadlines, we were able to have a strong basis for the project days
  before the due date.
- Testing the functions was also immensely useful for our group to know where
  the bugs were coming from. Personally, I had never used unittest.mock
  before and it was exciting to learn a tool that was so useful. Especially
  when working in groups, testing allows us to locate whose code is causing 
  the bug so that we do not do a wild goose chase to find an issue that one
  of us could fix on our own.
- Related to the last bullet point, understanding the code that was given to
  us and its return values is also essential to writing good code. It's 
  really important to document the expected return values of your code, so
  that others can account for the different cases in the future. 


# Demo
![Particle filter demo](particle_filter.gif)
