## Parameters

Description of the node parameters and how to tune them.

**This document is work-in-progress.**

## Likelihood calculation

- **match\_dist\_min** (float, default: `0.2`, unit: meter):\
    Point to map distance larger than *match\_dist\_min* is ignored in the likelihood calculation.
- **match\_dist\_flat** (float, default: `0.05`, unit: meter):\
    Point-to-map distance smaller than *match\_dist\_flat* is clipped to *match\_dist\_flat*. Set to three-sigma of the input pointcloud noise or larger to avoid fixing to local minimums.

## Expansion resetting

- **match\_ratio\_thresh** (float, default: `0.0`, unit: ratio):\
    Trigger the expansion resetting if the ratio of the measured points which have corresponding map points is smaller than *match\_ratio\_thresh*. Set 0.0 to disable the expansion resetting.
- **expansion\_var\_(x|y|z)** (float, default: `0.2`, unit: meter):\
    Gaussian noise distribution to expand the particle distribution on the expansion resetting.
- **expansion\_var\_(roll|pitch|yaw)** (float, default: `0.05`, unit: radian):\
    Gaussian noise distribution to expand the particle distribution on the expansion resetting.

## Motion prediction

- **odom\_err\_lin\_lin** (float, default: `0.1`, unit: meter/meter):\
    Standard deviation of the translational error per unit of the translational component of the robot's motion.
- **odom\_err\_lin\_ang** (float, default: `0.05`, unit: radian/meter):\
    Standard deviation of the rotational error per unit of the translational component of the robot's motion.
- **odom\_err\_ang\_lin** (float, default: `0.05`, unit: meter/radian):\
    Standard deviation of the translational error per unit of the rotational component of the robot's motion.
- **odom\_err\_ang\_ang** (float, default: `0.05`, unit: radian/rad):\
    Standard deviation of the rotational error per unit of the rotational component of the robot's motion.
- **odom\_err\_integ\_lin\_tc** (float, default: `10.0`, unit: second):\
    Time constant to forget the integral of the translational odometry error.
- **odom\_err\_integ\_lin\_sigma** (float, default: `100.0`, unit: meter):\
    Acceptable range of the integral of the translational odometry error.
- **odom\_err\_integ\_ang\_tc** (float, default: `10.0`, unit: second):\
    Time constant to forget the integral of the rotational odometry error.
- **odom\_err\_integ\_ang\_sigma** (float, default: `100.0`, unit: radian):\
    Acceptable range of the integral of the rotational odometry error.

## ROS communication

- **odom\_queue\_size** (int, default: `200`, unit: count):\
    Odometry subscriber queue size
- **imu\_queue\_size** (int, default: `200`, unit: count):\
    IMU subscriber queue size
- **cloud\_queue\_size** (int, default: `100`, unit: count):\
    Point cloud subscriber queue size
