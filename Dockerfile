FROM ros:kinetic

RUN apt-get update && \
		apt-get install -y --no-install-recommends sudo ros-kinetic-pcl-ros libgtest-dev wget && \
		rm -rf /var/lib/apt/lists/*

RUN rosdep update && \
	mkdir -p /catkin_ws/src && \
	bash -c "cd /catkin_ws/src && . /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace && cd .. && catkin_make"

COPY ${DATASET_CACHE_DIR}/${DATASET_FILE} /catkin_ws/build/mcl_3dl/test/${DATASET_FILE}
COPY ${DATASET_CACHE_DIR}/${DATASET_REF_FILE} /catkin_ws/build/mcl_3dl/test/${DATASET_REF_FILE}

COPY ./ /catkin_ws/src/mcl_3dl
RUN /catkin_ws/src/mcl_3dl/.test-scripts/test.sh



