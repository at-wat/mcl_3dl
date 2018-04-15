cmake_minimum_required(VERSION 2.8.3)
project(mcl_3dl)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  pcl_ros
	roscpp
  roslint
  rostest
  sensor_msgs
  std_msgs
  std_srvs
  tf
  visualization_msgs
)
find_package(PCL 1.7 REQUIRED)

add_service_files(
  FILES ResizeParticle.srv
)
add_message_files(
  FILES Status.msg
)
generate_messages(
  DEPENDENCIES std_msgs
)

catkin_package(
	CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf
    visualization_msgs
)
find_package(Boost REQUIRED COMPONENTS chrono REQUIRED)
find_package(Eigen3 REQUIRED)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")


include_directories(${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIR} include)
link_directories(${Boost_LIBRARY_DIRS} ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Workaround for the bug in PCL(<1.8.1) https://github.com/PointCloudLibrary/pcl/issues/1406
remove_definitions(-DDISABLE_LIBUSB-1.0)

add_executable(mcl_3dl src/mcl_3dl.cpp)
target_link_libraries(mcl_3dl ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(mcl_3dl mcl_3dl_generate_messages)


roslint_cpp()

if(CATKIN_ENABLE_TESTING)
	add_subdirectory(test)
  roslint_add_test()
endif()

install(TARGETS mcl_3dl
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

