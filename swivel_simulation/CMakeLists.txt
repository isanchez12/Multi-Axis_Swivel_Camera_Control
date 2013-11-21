


cmake_minimum_required(VERSION 2.8.3)
project(irk_simulation)

find_package(catkin REQUIRED COMPONENTS 
  roscpp gazebo_ros_control gazebo_ros) 
#effort_controllers gazebo_ros)

# Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

catkin_package(
  CATKIN_DEPENDS roscpp gazebo_ros_control effort_controllers gazebo_ros
  INCLUDE_DIRS # TODO include
  LIBRARIES # TODO
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# Build RobotSim Interface
add_library(${PROJECT_NAME}
  src/swivel_sim.cpp )
