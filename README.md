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

## Running the demo

The example bag file of 2+4-DOF tracked vehicle with two Hokuyo YVT-X002 3-D LIDAR is available online.
Pre-processed (filtered) 3-D pointcloud, IMU pose, odometry, and map data are packed in the bag.

```.sh
# Download the example bag (230M)
wget -P ~/Downloads https://openspur.org/~atsushi.w/dataset/mcl_3dl/short_test.bag

# Running the demo
roslaunch mcl_3dl test.launch use_pointcloud_map:=false use_cad_map:=false \
  use_bag_file:=true bag_file:=${HOME}/Downloads/short_test.bag
```

The map data in the bag was generated by using the [cartographer_ros](https://github.com/googlecartographer/cartographer_ros) and filtered by using pcl_outlier_removal and pcl_voxel_grid utilities.


![Rviz image of the demo](https://github.com/at-wat/mcl_3dl/blob/master/doc/images/demo_rviz.jpg?raw=true)

MarkerArray shows several *mcl_3dl* internal information.
- Purple spheres: sampled points used in the likelihood-model calculation
- Red lines: casted rays in the beam-model calculation
- Red boxes: detected collisions in raycasting


[The demo without odometry](doc/DemoWithoutOdometry.md) is also available.

## License

*mcl_3dl* is available under BSD license.
