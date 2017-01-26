# mcl_3dl

## Package summary

*mcl_3dl* is a ROS node to perform a probabilistic 3-D/6-DOF localization system for mobile robots with 3-D LIDAR(s).
It implements pointcloud based Monte Carlo localization that uses a reference pointcloud as a map.

The node receives the reference pointcloud as an environment map and localizes 6-DOF (x, y, z, yaw, pitch, roll) pose of measured pointclouds assisted by a motion prediction using odometry.

Currently, the supported motion model is differential-wheeled-robot.
The node provides classic MCL; currently, it doesn't implement adaptive feature like KDL-sampling and etc.

## Algorithms

A fundamental algorithm of *mcl_3dl* node is Monte Carlo localization (MCL), aka particle filter localization.
MCL represents a probabilistic distribution of estimated pose as density and weight of particles and estimates the pose from the distribution.

See [details](doc/Algorithms.md).

## TODO

* Re-localization: [Expansion resetting](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1389781) might be implemented in future.
