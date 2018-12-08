cmake_minimum_required(VERSION 2.8.3)
project(birl_trajectory_excution)


find_package(catkin REQUIRED
    COMPONENTS
    actionlib
    rospy
    trajectory_msgs
    baxter_interface
    baxter_core_msgs
    sensor_msgs
)

catkin_python_setup()