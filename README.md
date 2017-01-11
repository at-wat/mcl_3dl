# mcl_3dl

## Package summary

mcl_3dl is a ROS node to perform a probabilistic 3-D/6-DOF localization system for mobile robots with LIDAR(s).
It implements pointcloud based Monte Carlo localization that uses a reference pointcloud as a map.

The node receives the reference pointcloud as an environment map and localizes 6-DOF (x, y, z, yaw, pitch, roll) pose of measured pointclouds assisted by a motion prediction using odometry.

Currently, the supported motion model is differential-wheeled-robot.
The node provides classic MCL; currently, it doesn't implement adaptive feature like KDL-sampling and etc.


## Algorithms

A fundamental algorithm of mcl_3dl node is Monte Carlo localization (MCL), aka particle filter localization.
MCL represents a probabilistic distribution of estimated pose as density and weight of particles.

### Measurement

A main aspect of the node is a combination of beam_range_finder_model, likelihood_field_range_finder_model that were well-described in the book Probabilistic Robotics, by Thrun, Burgard, and Fox.
beam_range_finder_model can reduce false positive matching by handling visibility of the measured point in the map.
However, beam_range_finder_model is heavier than likelihood_field_range_finder_model.

mcl_3dl node uses both of them to perform good matching result and also light computation power.
The node calculates a likelihood of each particle by multiplying likelihood of likelihood_field_range_finder_model by using a larger number of points and one of beam_range_finder_model by using only a few random sampled points.
It realizes pointcloud matching with small computation power with rejecting matched-but-wrong matches.

In this implementation, the likelihood of likelihood_field_range_finder_model is a summation of distances from each measured point to closest point in the map. 
The closest point search uses Kd-tree, and the measured point is voxel filtered and random sampled to reduce computation power.

Likelihood of beam_range_finder_model is calculated by ray casting.
Where `N` is a number of rays of measured point, `n` is a number of the rays which passes through objects described in the map, and *alpha* is a rejection weight, likelihood is given as `alpha^(n/N)`.

### Prediction

The pose of the particles is predicted according to odometry data with noise parameters.

### Resampling
