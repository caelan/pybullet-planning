import random
import numpy as np

from ..utils import matrix_from_quat, point_from_pose, quat_from_pose, quat_from_matrix, get_joint_limits, get_joint_position

# TODO: lookup robot & tool in dictionary and use if exists


def compute_forward_kinematics(fk_fn, conf):
    pose = fk_fn(list(conf))
    pos, rot = pose
    quat = quat_from_matrix(rot) # [X,Y,Z,W]
    return pos, quat


def compute_inverse_kinematics(ik_fn, pose, sampled=[]):
    pos = point_from_pose(pose)
    rot = matrix_from_quat(quat_from_pose(pose)).tolist()
    if sampled:
        solutions = ik_fn(list(rot), list(pos), sampled)
    else:
        solutions = ik_fn(list(rot), list(pos))
    if solutions is None:
        return []
    return solutions


def get_ik_limits(robot, joint, limits=False):
    if limits is False:
        return get_joint_limits(robot, joint)
    elif limits is None:
        value = get_joint_position(robot, joint)
        return value, value
    return limits


def select_solution(solutions, nearby_conf=None, **kwargs):
    if not solutions:
        return None
    if nearby_conf is None:
        return random.choice(solutions)
    # TODO: search over neighborhood of sampled joints when nearby_conf != None
    return min(solutions, key=lambda conf: np.linalg.norm(np.array(conf) - np.array(nearby_conf), **kwargs))
