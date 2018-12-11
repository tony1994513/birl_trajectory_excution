#!/usr/bin/env python

import baxter_interface
import rospy
from birl_trajectory_excution.robot_runing_mode import moving_point_mode
import ipdb

if __name__ == '__main__':
    rospy.init_node("set_right_arm_py")
    rospy.sleep(0.2)
    joints = [-0.3708398554712988, -1.1742622931262843, -0.09012137128826805, 1.4208497047788644,
      0.04141748127290617, 1.3111700784450575, -1.352971054914935]
    moving_point_mode(joints)
    print "Done"




