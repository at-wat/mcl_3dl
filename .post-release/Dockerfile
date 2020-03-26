ARG ROS_DISTRO=kinetic
FROM ros:${ROS_DISTRO}-ros-core

RUN apt-get -qq update \
  && apt-get upgrade -y \
  && apt-get install -y --no-install-recommends \
    wget \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/* \
  && wget -q -P / https://openspur.org/~atsushi.w/dataset/mcl_3dl/short_test3.bag

RUN echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list \
  && rosdep update

ARG CACHE_CLEAN

RUN apt-get -qq update \
  && apt-get upgrade -y \
  && apt-get install -y --no-install-recommends \
    bc \
    ros-${ROS_DISTRO}-mcl-3dl \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-tf \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY test.sh /
ENTRYPOINT ["/test.sh"]
