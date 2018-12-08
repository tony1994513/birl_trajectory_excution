from joint_action_client import Trajectory
from utils import get_current_angle
from _constant import limb

def moving_point_mode(point):
    rospy.loginfo("Move point mode\n")
    traj = Trajectory(limb)
    cur_angle = get_current_angle()
    start_wait_time = traj.find_start_offset(point,cur_angle,speed=robot_runing_speed)
    traj.clear(limb)
    traj.add_point(cur_angle, 0.0)
    traj.add_point(point,start_wait_time) 
    traj.start()
    traj.wait(start_wait_time)

def moving_trajectory_mode(command_angles,gripper_state="open"): 
    traj = Trajectory(limb)
    cur_angle = get_current_angle()
    moving_point_mode(command_angles[0])
    traj_wait_time = traj.find_offset(command_angles,speed=robot_runing_speed)

    for idx, command in enumerate(command_angles):
        wait_time =  traj_wait_time[idx]
        traj.add_point(command,wait_time)
    traj.start()
    traj.wait(wait_time)
    rospy.loginfo("Moving trajectory mode\n")

    if gripper_state == "open":
        traj.gripper_open()

    elif gripper_state == "close":
        rospy.sleep(1)
        traj.gripper_close()

def Moving_moveit_mode():
    pass