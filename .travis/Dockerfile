ARG ROS_DISTRO=kinetic
# ========================================
FROM alpine:3.7 as cloner

RUN apk add --no-cache git py-pip \
  && pip install wstool

WORKDIR /repos
COPY .rosinstall /repos/deps.rosinstall

RUN wstool init src --shallow deps.rosinstall
COPY . /repos/src/mcl_3dl

RUN mkdir -p /repos-manifests/src
RUN find . -name package.xml | xargs -ISRC cp --parents SRC /repos-manifests/

# ========================================
FROM ros:${ROS_DISTRO}-ros-core

RUN apt-get -qq update \
  && apt-get upgrade -y \
  && apt-get install -y --no-install-recommends \
    curl \
    libxml2-utils \
    python-pip \
    sudo \
    wget \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY --from=cloner /repos-manifests/src /catkin_ws/src
RUN rosdep update \
  && apt-get -qq update \
  && rosdep install --from-paths /catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*
RUN pip install gh-pr-comment catkin_lint

COPY --from=cloner /repos/src /catkin_ws/src

RUN cd /catkin_ws/src \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && catkin_init_workspace
