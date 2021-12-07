cmake_minimum_required(VERSION 3.1.3)
project(mcl_3dl)

set(CATKIN_DEPENDS
  roscpp

  pcl_ros
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs

  diagnostic_updater
  geometry_msgs
  mcl_3dl_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS})
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8.0 REQUIRED
  COMPONENTS
    core
    filters
    kdtree
)
find_package(Boost REQUIRED COMPONENTS chrono)
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDS}
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIR})
add_definitions(${PCL_DEFINITIONS})


# Workaround for debian stretch build (https://bugs.debian.org/cgi-bin/bugreport.cgi?bug=894656)
list(REMOVE_ITEM PCL_LIBRARIES
  "vtkGUISupportQt"
  "vtkGUISupportQtOpenGL"
  "vtkGUISupportQtSQL"
  "vtkGUISupportQtWebkit"
  "vtkViewsQt"
  "vtkRenderingQt"
)

# Workaround for the bug in PCL<1.8.1 (https://github.com/PointCloudLibrary/pcl/issues/1406)
remove_definitions(-DDISABLE_LIBUSB-1.0)

# Workaround for PCL-1.7 problem on C++11 (https://github.com/PointCloudLibrary/pcl/issues/619)
# Debug build without optimization causes Segmentation fault.
# Set default build type to Release and add -O1 to CXX_FLAGS_DEBUG.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")

# Binary installed pcl provided by Linux distro is built with -march=native
# which causes a lot of compatibility problems.
# Define PCL_NO_PRECOMPILE to disable using the binary version.
add_definitions(-DPCL_NO_PRECOMPILE)


add_executable(mcl_3dl
  src/cloud_accum.cpp
  src/lidar_measurement_model_beam.cpp
  src/lidar_measurement_model_likelihood.cpp
  src/mcl_3dl.cpp
  src/parameters.cpp
  src/point_types.cpp
)
target_link_libraries(mcl_3dl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(mcl_3dl ${catkin_EXPORTED_TARGETS})


if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(rosunit REQUIRED)
  add_subdirectory(test)

  if(NOT "$ENV{ROS_DISTRO}" STREQUAL "melodic")
    find_package(roslint REQUIRED)
    roslint_cpp()
    roslint_add_test()
  endif()
endif()


install(TARGETS mcl_3dl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
