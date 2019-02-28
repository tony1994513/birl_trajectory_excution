#!/usr/bin/env python
import rospy
import os,sys
import ipdb
import numpy as np
import smach
import smach_ros

from birl_trajectory_planner.trajectory_planner import planner
from birl_inverse_kinematic import trac_ik_solver
from birl_trajectory_excution import joint_action_client,_constant,utils
from birl_kitting_experiment_simulation.gazebo_model_util import get_model_pose
from birl_trajectory_excution.utils import get_current_pose_list
from birl_trajectory_excution.robot_runing_mode import moving_point_mode, moving_trajectory_mode

limb = "right"
speed = 0.5
running_mode = "point" #option: point, trajectory
preplace_pose = [0.19, -0.8, 0.3, 1, 0, 0, 0]
place_pose = [0.19, -0.8, -0.105, 1, 0, 0, 0]
planner_type = "cart_trajectory_action_server"

class MoveToReadyPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        ready_pose = [ 0.03681553890924993, -0.9836651802315216,0.2515728492132078, 1.1443496677625187, -0.1054611791671222, 1.3698448435816746, -0.5744758050630875]
        if running_mode == "point":
            moving_point_mode(point=ready_pose,limb=limb,speed=speed ,gripper_state="open")
        elif running_mode == "trajectory":
            pass
            # moving_trajectory_mode(command_angles,gripper_state="open")
        return "succuss"

class MoveToHoverPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):

        start = get_current_pose_list()
        end =   get_model_pose("box_male",hover_flag=True)
        
        joint = planner(start, end, limb=limb,planner_type=planner_type,point_mode=True)
        if running_mode == "point":
            moving_point_mode(point=joint,limb=limb,speed=speed ,gripper_state="open")
        elif running_mode == "trajectory":
            pass
        return "succuss"

class MoveToPickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
   
        start = get_current_pose_list()
        end = get_model_pose("box_male")
        joint = planner(start, end, limb=limb,planner_type=planner_type,point_mode=True)
        if running_mode == "point":
            moving_point_mode(point=joint,limb=limb,speed=speed,gripper_state="close")
        elif running_mode == "trajectory":
            pass
        return "succuss"
        
class BackToPrePickPosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):

        start = get_current_pose_list()
        end = get_model_pose("box_male",hover_flag=True)
        joint = planner(start, end, limb=limb,planner_type=planner_type,point_mode=True)
        if running_mode == "point":
            moving_point_mode(point=joint,limb=limb,speed=speed,gripper_state="close")
        elif running_mode == "trajectory":
            pass
        return "succuss"

class MoveToPrePlacePosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        start = get_current_pose_list()
        end = preplace_pose
        joint = planner(start, end, limb=limb,planner_type=planner_type,point_mode=True)
        if running_mode == "point":
            moving_point_mode(point=joint,limb=limb,speed=speed,gripper_state="close")
        return "succuss"

class MoveToPlacePosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        start = get_current_pose_list()
        end = place_pose
        joint = planner(start, end, limb=limb,planner_type=planner_type,point_mode=True)
        if running_mode == "point":
            moving_point_mode(point=joint,limb=limb,speed=speed,gripper_state="open")
        return "succuss"

class BackToPrePlacePosition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        start = get_current_pose_list()
        end = preplace_pose
        joint = planner(start, end, limb=limb,planner_type=planner_type,point_mode=True)
        if running_mode == "point":
            moving_point_mode(point=joint,limb=limb,speed=speed,gripper_state="open")
        return "succuss"

class MoveBackToReadyPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succuss'])

    def execute(self, userdata):
        ready_pose = [ 0.03681553890924993, -0.9836651802315216,0.2515728492132078, 1.1443496677625187, -0.1054611791671222, 1.3698448435816746, -0.5744758050630875]
        if running_mode == "point":
            moving_point_mode(point=ready_pose,limb=limb,speed=speed,gripper_state="open")
        return "succuss"

def main():
    rospy.init_node("test_robot_running",anonymous=True)
    rospy.sleep(0.2) # wait for server to start up

    sm = smach.StateMachine(outcomes=['Done'])
    with sm:
        smach.StateMachine.add(MoveToReadyPose.__name__, MoveToReadyPose(), 
                               transitions={'succuss':MoveToHoverPosition.__name__})     

        smach.StateMachine.add(MoveToHoverPosition.__name__, MoveToHoverPosition(), 
                               transitions={'succuss':MoveToPickPosition.__name__})    

        smach.StateMachine.add(MoveToPickPosition.__name__, MoveToPickPosition(), 
                               transitions={'succuss':BackToPrePickPosition.__name__}),      

        smach.StateMachine.add(BackToPrePickPosition.__name__, BackToPrePickPosition(), 
                               transitions={'succuss':MoveToPrePlacePosition.__name__})    
                                                                                   
        smach.StateMachine.add(MoveToPrePlacePosition.__name__, MoveToPrePlacePosition(), 
                               transitions={'succuss':MoveToPlacePosition.__name__})

        smach.StateMachine.add(MoveToPlacePosition.__name__, MoveToPlacePosition(), 
                               transitions={'succuss':BackToPrePlacePosition.__name__})

        smach.StateMachine.add(BackToPrePlacePosition.__name__, BackToPrePlacePosition(), 
                               transitions={'succuss':MoveBackToReadyPose.__name__})

        smach.StateMachine.add(MoveBackToReadyPose.__name__, MoveBackToReadyPose(), 
                               transitions={'succuss':'Done'})
                               

    outcome = sm.execute()


if __name__ == '__main__':
    sys.exit(main())