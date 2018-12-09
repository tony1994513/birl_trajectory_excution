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

catkin_package(
 CATKIN_DEPENDS trajectory_msgs sensor_msgs  std_msgs baxter_core_msgs smach actionlib)

 include_directories(
  ${catkin_INCLUDE_DIRS}
)