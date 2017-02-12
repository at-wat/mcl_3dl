FROM ros:kinetic

RUN apt-get update && \
		apt-get install -y --no-install-recommends sudo ros-kinetic-pcl-ros libgtest-dev wget && \
		rm -rf /var/lib/apt/lists/*

RUN rosdep update && \
	mkdir -p /catkin_ws/src && \
	bash -c "cd /catkin_ws/src && . /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace && cd .. && catkin_make"

RUN mkdir -p /catkin_ws/build/mcl_3dl/test/ && \
	wget --quiet https://openspur.org/~atsushi.w/dataset/mcl_3dl/short_test_ref.topic -O /catkin_ws/build/mcl_3dl/test/short_test_ref.topic && \
	wget --quiet https://openspur.org/~atsushi.w/dataset/mcl_3dl/short_test.bag -O /catkin_ws/build/mcl_3dl/test/short_test.bag


COPY ./ /catkin_ws/src/mcl_3dl
RUN /catkin_ws/src/mcl_3dl/script.docker/test.sh



