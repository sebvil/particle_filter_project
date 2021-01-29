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

