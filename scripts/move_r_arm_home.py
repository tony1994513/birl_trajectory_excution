#!/usr/bin/env python

import baxter_interface
import rospy
from birl_trajectory_excution.robot_runing_mode import moving_point_mode
import ipdb

if __name__ == '__main__':
    rospy.init_node("right_arm_moving_point")
    rospy.sleep(0.2)
    desired_point = [ 0.03681553890924993, -0.9836651802315216,0.2515728492132078, 1.1443496677625187,
     -0.1054611791671222, 1.3698448435816746, -0.5744758050630875]
    moving_point_mode(point=desired_point,limb="right",speed=0.5,gripper_state="open")
    print "Done"




