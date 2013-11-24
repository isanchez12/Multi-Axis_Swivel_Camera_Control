cmake_minimum_required(VERSION 2.8.3)
project(swivel_simulation)

find_package(catkin REQUIRED COMPONENTS gazebo_ros) 
#effort_controllers gazebo_ros)


catkin_package(
  CATKIN_DEPENDS 
  INCLUDE_DIRS # TODO include
  LIBRARIES # TODO
  )

