#!/usr/bin/env python
"""
service client module. create a class 
which contains a variety of service client fucntions
"""

import sys
import rospy
import rospkg
import copy
import struct
import numpy as np
import actionlib

from birl_sim_examples.srv import *

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
import operator
import baxter_interface
import ipdb
from _constant import robot_runing_speed,limb_name

class Trajectory(object):
    def __init__(self, limb,verbose = False):
        # set up the action client
        rospy.sleep(0.2)
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        
        ## check up if the joint_trajectory action server setup
        ## please rosrun baxter_interface joint_trajectory_server
        server_up = self._client.wait_for_server(timeout=rospy.Duration(5.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

        #clear all  JointTrajectoryPoint
        self.clear(limb)
        self._verbose = verbose
        # enable the IK Service
        ik_ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ik_ns, SolvePositionIK)
        rospy.wait_for_service(ik_ns, 5.0)
        
        self._gripper = baxter_interface.Gripper(limb)
        self._limb = baxter_interface.limb.Limb(limb)
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        if self._init_state != True:
            print("Enabling robot... ")
            self._rs.enable()

    def find_start_offset(self,start_angle, cur_angle,speed):
        dflt_vel = [speed] * 7
        diffs = map(operator.sub, start_angle, cur_angle)
        diffs = map(operator.abs, diffs)
        #determine the largest time offset necessary across all joints
        t_offset = max(map(operator.div, diffs, dflt_vel))
        return t_offset

    def find_offset(self, dmp_angle_plans,speed):
        dflt_vel = [speed]*7
        mat = np.array(dmp_angle_plans)
        last = mat[0]
        new_mat = [last]
        t_offset = []
        for idx in range(mat.shape[0]):
            diffs = map(operator.sub, mat[idx], last)
            diffs = map(operator.abs, diffs)
            t_offset_ = max(map(operator.div, diffs, dflt_vel))
            last = mat[idx]
            t_offset.append(t_offset_)
        return np.array(t_offset).cumsum()

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy.copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        return self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def add_pose_point(self,pose,time):
        angles = self.ik_request(pose)
        if not angles:
            return 0
        else:
            self.add_point(angles,time)
            return 1

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = limb_name

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)



