## Demo without odometry (very experimental)

With `fake_odom:=true`, *mcl_3dl* node works without `odom` topic.
Increasing number of the particles and noise variance enables localization without odometry.
Currently, this feature is very experimental and should be improved in the future.

Download the example bag and run the demo with following arguments.

```.sh
# Running the demo without odometry
roslaunch mcl_3dl test.launch use_pointcloud_map:=false use_cad_map:=false \
  use_bag_file:=true bag_file:=${HOME}/Downloads/short_test3.bag \
  without_odom:=true
```

## Demo without IMU (very experimental)

With `fake_imu:=true`, *mcl_3dl* node works without `imu/data` topic.

Download the example bag and run the demo with following arguments.

```.sh
# Running the demo without odometry
roslaunch mcl_3dl test.launch use_pointcloud_map:=false use_cad_map:=false \
  use_bag_file:=true bag_file:=${HOME}/Downloads/short_test3.bag \
  without_imu:=true
```
