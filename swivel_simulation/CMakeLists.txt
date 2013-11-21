cmake_minimum_required(VERSION 2.8.3)
project(swivel_simulation)

find_package(catkin REQUIRED COMPONENTS gazebo_ros) 
#effort_controllers gazebo_ros)

# Find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

catkin_package(
  CATKIN_DEPENDS 
  INCLUDE_DIRS # TODO include
  LIBRARIES # TODO
  )

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

# Build RobotSim Interface
add_library(${PROJECT_NAME}
  src/swivel_sim.cpp )
