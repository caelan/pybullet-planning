import random

from ..utils import get_ik_limits, compute_forward_kinematics, compute_inverse_kinematics, select_solution
from ...utils import multiply, get_link_pose, \
    link_from_name, get_joint_positions, invert, violates_limits, joint_from_name, joints_from_names
from ...eth_rfl_utils import get_torso_arm_joints

BASE_FRAME = 'gantry_xy_carriage'
IK_FRAME = 'robot_tool0'
TOOL_FRAME = 'eef_tcp_frame' #robot_tool0 | eef_tcp_frame

GANTRY_JOINTS = {
    # 'x': 'gantry_x_joint',
    # 'y': 'gantry_y_joint',
    'z': 'gantry_z_joint'
}

#####################################

def get_tool_pose(robot):
    from .ikfast_eth_rfl import get_fk
    ik_joints = get_torso_arm_joints(robot, 'right')
    conf = get_joint_positions(robot, ik_joints)
    # TODO: this should be linked to ikfast's get numOfJoint junction
    base_from_ik = compute_forward_kinematics(get_fk, conf)
    base_from_tool = multiply(base_from_ik, invert(get_tool_from_ik(robot)))

    world_from_base = get_link_pose(robot, link_from_name(robot, BASE_FRAME))
    return multiply(world_from_base, base_from_tool)

#####################################

def is_ik_compiled():
    try:
        from .ikfast_eth_rfl import ikfast_eth_rfl
        return True
    except ImportError:
        return False

def get_ik_generator(robot, tool_pose, gantry_z_limits=False):
    from .ikfast_eth_rfl import get_ik
    world_from_base = get_link_pose(robot, link_from_name(robot, BASE_FRAME))
    base_from_tool = multiply(invert(world_from_base), tool_pose)
    base_from_ik = multiply(base_from_tool, get_tool_from_ik(robot))

    sampled_limits = get_ik_limits(robot, joint_from_name(robot, GANTRY_JOINTS['z']), gantry_z_limits)

    while True:
        sampled_values = [random.uniform(*sampled_limits)]
        ik_joints = get_torso_arm_joints(robot, 'right')
        confs = compute_inverse_kinematics(get_ik, base_from_ik, sampled_values)
        yield [q for q in confs if not violates_limits(robot, ik_joints, q)]

def get_tool_from_ik(robot):
    world_from_tool = get_link_pose(robot, link_from_name(robot, TOOL_FRAME))
    world_from_ik = get_link_pose(robot, link_from_name(robot, IK_FRAME))
    # tool from the bare flange (6th axis)
    return multiply(invert(world_from_tool), world_from_ik)

def sample_tool_ik(robot, tool_pose, nearby_conf=None, max_attempts=10, **kwargs):
    generator = get_ik_generator(robot, tool_pose, **kwargs)
    for _ in range(max_attempts):
        try:
            solutions = next(generator)
            if solutions:
                return select_solution(solutions, nearby_conf=nearby_conf)
        except StopIteration:
            break
    return None
