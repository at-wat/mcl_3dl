FROM ros:kinetic

RUN apt-get -qq update && \
	apt-get install -y --no-install-recommends sudo libeigen3-dev libpcl-dev libproj-dev libqtgui4 libgtest-dev wget curl python-pip && \
	apt-get clean && \
	rm -rf /var/lib/apt/lists/*

RUN rosdep update && \
	mkdir -p /catkin_ws/src && \
	bash -c "cd /catkin_ws/src && . /opt/ros/${ROS_DISTRO}/setup.bash && catkin_init_workspace && cd .. && catkin_make"

COPY ./.travis.scripts/install_deps.sh /catkin_ws/src/mcl_3dl/.travis.scripts/install_deps.sh
COPY ./package.xml /catkin_ws/src/mcl_3dl/package.xml
RUN /catkin_ws/src/mcl_3dl/.travis.scripts/install_deps.sh

ARG TRAVIS_PULL_REQUEST=false
ARG TRAVIS_PULL_REQUEST_SLUG=""
ARG TRAVIS_BOT_GITHUB_TOKEN=""

COPY . /catkin_ws/src/mcl_3dl
