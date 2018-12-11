#!/usr/bin/env python

import baxter_interface
import rospy
from birl_trajectory_excution.robot_runing_mode import moving_point_mode
import ipdb

if __name__ == '__main__':
    rospy.init_node("set_right_arm_py")
    rospy.sleep(0.2)
    joints = [  0.7466651485032252, -1.0803059698683026,-0.011888351106111956, 1.4998497153549633,
     0.04908738521233324, 1.1098351000350968, 0.01112136071216925]
    moving_point_mode(joints)
    print "Done"




