cmake_minimum_required(VERSION 2.8.3)
project(mcl_3dl)

find_package(catkin REQUIRED COMPONENTS
	roscpp std_msgs message_generation sensor_msgs pcl_ros
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
	CATKIN_DEPENDS roscpp std_msgs sensor_msgs pcl_ros
	DEPENDS system_lib
)

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


include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIR} src)
link_directories(${PCL_LIBRARY_DIR})
add_definitions(${PCL_DEFINITIONS})

add_executable(mcl_3dl src/mcl_3dl.cpp)
target_link_libraries(mcl_3dl ${catkin_LIBRARIES} ${PCL_LIBRARIES})

