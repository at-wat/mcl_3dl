FROM ros:kinetic

RUN apt-get update && \
		apt-get install -y --no-install-recommends sudo ros-kinetic-pcl-ros && \
		rm -rf /var/lib/apt/lists/*

RUN apt-get update && \
		apt-get install -y --no-install-recommends libgtest-dev && \
		rm -rf /var/lib/apt/lists/*

COPY ./script.docker /tmp/mcl_3dl/script.docker
RUN /tmp/mcl_3dl/script.docker/init.sh

COPY ./ /catkin_ws/src/mcl_3dl
RUN /tmp/mcl_3dl/script.docker/test.sh



