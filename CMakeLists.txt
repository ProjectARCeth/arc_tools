cmake_minimum_required(VERSION 2.8.3)
project(arc_tools)


find_package(catkin_simple 0.1.0 REQUIRED COMPONENTS
  	roscpp
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs
    eigen_catkin
    catkin_simple REQUIRED
)

catkin_simple(ALL_DEPS_REQUIRED)

cs_add_library(${PROJECT_NAME} 
    src/coordinate_transform.cpp
	src/state_and_path_publisher.cpp
	src/kalman_filter.cpp
    src/signal_filter.cpp
)

#include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(rpy_transformation src/test/rpy_transformation.cpp)
target_link_libraries(rpy_transformation ${catkin_LIBRARIES} ${PROJECT_NAME})

cs_install()
cs_export()
