cmake_minimum_required(VERSION 2.8.3)
project(agent)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## This macro ensures modules and global scripts declared therein get installed
catkin_python_setup()

catkin_package()