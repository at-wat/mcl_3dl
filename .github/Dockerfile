# This file is automatically deployed from https://github.com/at-wat/.rospkg-assets.
# Please don't directly edit; update at-wat/.rospkg-assets instead.

ARG ROS_DISTRO=noetic
# ========================================
FROM alpine:latest as cloner

RUN apk add --no-cache git py3-pip \
  && python3 -m pip install wstool

WORKDIR /repos-isolated
COPY .rosinstall-isolated /repos-isolated/deps.rosinstall

RUN wstool init src --shallow deps.rosinstall

RUN mkdir -p /repos-isolated-manifests/src
RUN find . -name package.xml | xargs -ISRC cp --parents SRC /repos-isolated-manifests/

WORKDIR /repos
COPY .rosinstall /repos/deps.rosinstall

RUN wstool init src --shallow deps.rosinstall
COPY . /repos/src/self

RUN mkdir -p /repos-manifests/src
RUN find . -name package.xml | xargs -ISRC cp --parents SRC /repos-manifests/

# ========================================
FROM ros:${ROS_DISTRO}-ros-core

RUN sed -e \
  's|http://archive\.ubuntu\.com|http://us-east-1.ec2.archive.ubuntu.com|' \
  -i /etc/apt/sources.list

ENV DEBIAN_FRONTEND=noninteractive

COPY --from=cloner /repos-manifests/src /catkin_ws/src
COPY --from=cloner /repos-isolated-manifests/src /catkin_ws_isolated/src
RUN if [ ${ROS_DISTRO} = "noetic" ]; then PYTHON_VERSION_SUFFIX=3; else PYTHON_VERSION_SUFFIX=; fi \
  && apt-get -qq update \
  && apt-get install -y --no-install-recommends \
    build-essential \
    curl \
    libxml2-utils \
    python${PYTHON_VERSION_SUFFIX}-catkin-pkg \
    python${PYTHON_VERSION_SUFFIX}-rosdep \
    python${PYTHON_VERSION_SUFFIX}-rosdep-modules \
    python${PYTHON_VERSION_SUFFIX}-rosdistro \
    python${PYTHON_VERSION_SUFFIX}-rospkg \
    python${PYTHON_VERSION_SUFFIX}-setuptools \
    sudo \
    wget \
  && rosdep init \
  && rosdep update \
  && rosdep install --from-paths /catkin_ws/src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && rosdep install --from-paths /catkin_ws_isolated/src --ignore-src --rosdistro=${ROS_DISTRO} -y \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

COPY --from=cloner /repos-isolated/src /catkin_ws_isolated/src
RUN cd /catkin_ws_isolated \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && catkin_make_isolated \
    --install --install-space=/opt/ros/${ROS_DISTRO} \
    -DCMAKE_BUILD_TYPE=Debug \
  && rm -rf build devel

COPY --from=cloner /repos/src /catkin_ws/src

RUN cd /catkin_ws/src \
  && . /opt/ros/${ROS_DISTRO}/setup.sh \
  && catkin_init_workspace

COPY .github/gh-pr-comment /usr/local/bin/
