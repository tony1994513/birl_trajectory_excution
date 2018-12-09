from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import numpy
from joint_action_client import Trajectory
from birl_trajectory_excution._constant import limb_name


def handle_object_in_gazebo_offset(pose_list):
    defalt_offset_z = 0.918
    pose_list[2] = pose_list[2] - defalt_offset_z
    return pose_list

def list_to_pose(_list):
    _pose = Pose()
    _pose.position.x = _list[0]
    _pose.position.y = _list[1]
    _pose.position.z = _list[2]
    _pose.orientation.x = _list[3]
    _pose.orientation.y = _list[4]
    _pose.orientation.z = _list[5]
    _pose.orientation.w = _list[6]
    return _pose

def pose_to_list(_pose):
    return [_pose.position.x,_pose.position.y,_pose.position.z,
    _pose.orientation.x,_pose.orientation.y,_pose.orientation.z,_pose.orientation.w]

def filter_static_points(mat):
    last = mat[0]
    new_mat = [last]
    for idx in range(mat.shape[0]):
        if numpy.linalg.norm(mat[idx]-last) < 0.01 \
            and idx != mat.shape[0]-1:
            pass
        else:
            new_mat.append(mat[idx])
            last = mat[idx]
    return numpy.array(new_mat)

def get_current_angle(limb="right"):
    traj = Trajectory(limb)
    limb_interface = traj._limb 
    cur_angle = [limb_interface.joint_angle(joint) for joint in limb_name]
    return cur_angle


def get_current_pose_list(limb="right"):
    traj = Trajectory(limb)
    limb_interface = traj._limb 
    current_pose_dic = limb_interface.endpoint_pose()
    current_pose_list = [ current_pose_dic['position'].x, 
                    current_pose_dic['position'].y,
                    current_pose_dic['position'].z,
                    current_pose_dic['orientation'].x,
                    current_pose_dic['orientation'].y,
                    current_pose_dic['orientation'].z,
                    current_pose_dic['orientation'].w]  
    return current_pose_list