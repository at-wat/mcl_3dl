## Demo without odometry (very experimental)

Even if without odometry, *mcl_3dl* node requires `/odom` topic that has orientation from IMU.
track_odometry node in [neonavigation meta-package](https://github.com/at-wat/neonavigation), which is designed to combine wheel odometry and IMU, can generate it.

Download the example bag and run the demo with following arguments.

```.sh
# Running the demo without odometry
roslaunch mcl_3dl test.launch use_pointcloud_map:=false use_cad_map:=false use_bag_file:=true bag_file:=${HOME}/Downloads/short_test.bag use_neonavigation:=true without_odom:=true
```

neonavigation meta-package is required to try this.
