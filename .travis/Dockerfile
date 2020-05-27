ARG ROS_DISTRO=kinetic
# ========================================
FROM alpine:latest as cloner

RUN apk add --no-cache git py3-pip \
  && python3 -m pip install wstool

WORKDIR /repos
COPY .rosinstall /repos/deps.rosinstall

RUN wstool init src --shallow deps.rosinstall
COPY . /repos/src/mcl_3dl

RUN mkdir -p /repos-manifests/src
RUN find . -name package.xml | xargs -ISRC cp --parents SRC /repos-manifests/

# ========================================
FROM ros:${ROS_DISTRO}-ros-core

RUN sed -e \
  's|http://archive\.ubuntu\.com|http://us-east-1.ec2.archive.ubuntu.com|' \
  -i /etc/apt/sources.list

COPY --from=cloner /repos-manifests/src /catkin_ws/src
RUN rosdep update \
  && apt-get -qq update \
  && apt-get install -y --no-install-recommends \
    curl \
    libxml2-utils \
    python3-pip \
    python3-setuptools \
    python3-rosdep \
    sudo \
    wget \
  && rosdep install --from-paths /catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*
RUN python3 -m pip install gh-pr-comment catkin_lint

COPY --from=cloner /repos/src /catkin_ws/src

RUN cd /catkin_ws/src \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && catkin_init_workspace
