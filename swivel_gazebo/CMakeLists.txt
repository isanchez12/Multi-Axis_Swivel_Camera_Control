cmake_minimum_required(VERSION 2.8.3)
project(swivel_gazebo)

find_package(catkin REQUIRED COMPONENTS gazebo_ros) 

catkin_package(
  CATKIN_DEPENDS 
  INCLUDE_DIRS # TODO include
  LIBRARIES # TODO
  )
