FROM ros:kinetic

RUN apt-get update && \
	apt-get install -y --no-install-recommends sudo libeigen3-dev libpcl-all libproj-dev libvtk5-dev libgtest-dev wget && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*

RUN rosdep update && \
	mkdir -p /catkin_ws/src && \
	bash -c "cd /catkin_ws/src && . /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace && cd .. && catkin_make"

COPY ./ /catkin_ws/src/mcl_3dl
RUN /catkin_ws/src/mcl_3dl/.travis.scripts/test.sh



