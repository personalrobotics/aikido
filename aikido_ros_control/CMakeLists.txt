cmake_minimum_required(VERSION 2.8.8)
project(aikido_ros_control)

find_package(catkin REQUIRED COMPONENTS control_msgs genmsg actionlib_msgs actionlib)
generate_messages(DEPENDENCIES actionlib_msgs control_msgs)