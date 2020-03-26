#!/bin/bash

source /opt/ros/${ROS_DISTRO}/setup.bash

set -eu
stdbuf -o L roslaunch mcl_3dl test.launch \
  use_pointcloud_map:=false \
  use_cad_map:=false \
  use_bag_file:=false &
pid=$!

sleep 5
rosrun tf tf_echo map base_link 1 | grep --line-buffered Translation | tee /tmp/pos.dat &
rosbag play --clock -r 2.5 /short_test3.bag &
pid_bag=$!

while true
do
  if [ ! -d "/proc/${pid_bag}" ]
  then
    break
  fi
  sleep 1
done

kill -SIGINT $pid
wait $pid

echo "========="
position=`tail -n1 /tmp/pos.dat | sed -e "s/^.*\[//;s/\].*$//;s/,//g"`
x=`echo $position | cut -f1 -d" "`
y=`echo $position | cut -f2 -d" "`
z=`echo $position | cut -f3 -d" "`
pos_error=`echo "sqrt(($x + 0.2)^2 + ($y + 0.17)^2 + ($z - 0.008))" | bc`
echo " - position error is $pos_error"
if [ `echo "$pos_error < 0.3" | bc` == "1" ]
then
  echo " SUCCESS"
else
  echo " ERROR"
  exit 1
fi
