# mcl_3dl

## Package summary

mcl_3dl is a ROS node to perform a probabilistic 3-D/6-DOF localization system for mobile robots with LIDAR(s).
It implements pointcloud based Monte Carlo localization that uses a reference pointcloud as a map.

The node receives the reference pointcloud as an environment map and localizes 6-DOF (x, y, z, yaw, pitch, roll) pose of measured pointclouds assisted by a motion prediction using odometry.

Currently, the supported motion model is differential-wheeled-robot.
The node provides classic MCL; currently, it doesn't implement adaptive feature like KDL-sampling and etc.


## Algorithms

A fundamental algorithm of the mcl_3dl node is Monte Carlo localization (MCL), aka particle filter localization.
MCL represents a probabilistic distribution of estimated pose as density and weight of particles.

### Measurement

A main aspect of the node is a combination of beam_range_finder_model, likelihood_field_range_finder_model that were well-described in the book Probabilistic Robotics, by Thrun, Burgard, and Fox.

### Prediction

The pose of the particles is predicted according to odometry data with noise given as parameters.

### Resampling
