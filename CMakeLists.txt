cmake_minimum_required(VERSION 2.8.3)
project(arc_tools)

find_package(catkin REQUIRED COMPONENTS
  	roscpp
    geometry_msgs
    sensor_msgs
    std_msgs
    eigen_catkin
    catkin_simple REQUIRED
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
	include/arc_tools/implementation/kalman_filter.cpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

cs_install()
cs_export()