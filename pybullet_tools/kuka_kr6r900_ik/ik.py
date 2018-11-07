from pybullet_tools.kuka_kr6r900_ik.ikfast_kuka_kr6r900 import get_ik, get_fk

from pybullet_tools.utils import matrix_from_quat, point_from_pose, quat_from_pose, quat_from_matrix, Pose, multiply, elapsed_time, \
    get_link_pose, link_from_name, joints_from_names, get_joint_positions, get_joint_limits, joint_from_name, invert, \
    get_joint_position, violates_limits, INF
# from pybullet_tools.pr2_utils import PR2_GROUPS, PR2_TOOL_FRAMES, arm_from_arm

import random

# As we only have one arm in kuka_kr6_r900,
# the default name is simply 'arm' (compared to 'left' and 'right' in PR2's case)
ARM_FK = {
    'arm': get_fk,
}

ARM_IK = {
    'arm': get_ik,
}

# the corresponding frame name can be found in
# models/kuka_kr6r900_description/framefab_kr6_r900_support/urdf/kuka_kr6_r900.urdf
IK_LINK = {
    'arm': 'robot_tool0',
}

BASE_LINK = 'robot_base_link'

# tool = robot_tool0 (6th axis flange)
# no extra dof

#def get_prefix(arm):
#    return arm[0]

def or_from_pyb_quat(quat):
    x, y, z, w = quat
    return [w, x, y, z]

def pyb_from_or_quat(quat):
    w, x, y, z = quat
    return [x, y, z, w]

def get_base_pose(robot):
    return get_link_pose(robot, link_from_name(robot, BASE_LINK))

def get_torso_arm_joints(robot, arm):
    return joints_from_names(robot, PR2_GROUPS['torso'] + PR2_GROUPS[arm_from_arm(arm)])

#####################################

def forward_kinematics(conf):
    '''
    compute fk.
    :param conf: robot's configuration
    :return:
    '''

    # TODO: this should be linked to ikfast's get numOfJoint function.
    assert len(conf) == 6

    arm = 'arm'
    fk_fn = ARM_FK[arm]

    pose = fk_fn(list(conf))
    pos, rot = pose
    quat = quat_from_matrix(rot) # [X,Y,Z,W]
    return pos, quat
    # switch from [r, i, j, k] to [i, j, k, r]
    # quat = quat if quat.real >= 0 else -quat  # solves q and -q being same rotation

def get_tool_pose(robot, arm):
    '''
    compute static transform from BASE_LINK -> IK_LINK
    :param robot:
    :param arm:
    :return:
    '''
    tool_pose = forward_kinematics(arm, get_joint_positions(robot, get_torso_arm_joints(robot, arm)))
    return multiply(get_base_pose(robot), tool_pose)

#####################################

def inverse_kinematics(pose):
    '''
    Compute IK. There is no free joint in this 6-dof robot.
    :param pose: end effector's pose
    :return:
    '''

    arm = 'arm'
    ik_fn = ARM_IK[arm]

    pos = point_from_pose(pose)
    rot = matrix_from_quat(quat_from_pose(pose)).tolist()
    solutions = ik_fn(list(rot), list(pos))
    if solutions is None:
        return []
    return solutions

# def get_limits(robot, joint, limits):
#     if limits is False:
#         return get_joint_limits(robot, joint)
#     elif limits is None:
#         torso_value = get_joint_position(robot, joint)
#         return torso_value, torso_value
#     return limits

#def sample_ik(robot, arm, target_pose, torso_limits, upper_limits):
#    arm_joints = get_arm_joints(robot, arm)
#    torso = random.uniform(*torso_limits)
#    upper = random.uniform(*upper_limits)
#    return [q for q in inverse_kinematics(arm, target_pose, torso, upper)
#           if not violates_limits(robot, arm_joints, q)]

# def get_ik_generator(robot, arm, world_pose, torso_limits=False, upper_limits=False):
#     target_pose = multiply(invert(get_base_pose(robot)), world_pose)
#     torso_limits = get_limits(robot, joint_from_name(robot, TORSO_JOINT), torso_limits)
#     upper_limits = get_limits(robot, joint_from_name(robot, UPPER_JOINT[arm]), upper_limits)
#     arm_joints = get_torso_arm_joints(robot, arm)
#     while True:
#         torso = random.uniform(*torso_limits)
#         upper = random.uniform(*upper_limits)
#         yield [q for q in inverse_kinematics(arm, target_pose, torso, upper)
#                if not violates_limits(robot, arm_joints, q)]

# TODO:
def sample_tool_ik(robot, arm, world_from_target, max_attempts=10, **kwargs):
    world_from_tool = get_link_pose(robot, link_from_name(robot, PR2_TOOL_FRAMES[arm]))
    world_from_ik = get_link_pose(robot, link_from_name(robot, IK_LINK[arm]))
    tool_from_ik = multiply(invert(world_from_tool), world_from_ik)
    ik_pose = multiply(world_from_target, tool_from_ik)
    generator = get_ik_generator(robot, arm, ik_pose, **kwargs)
    for _ in range(max_attempts):
        try:
            solutions = next(generator)
            if solutions:
                return random.choice(solutions)
        except StopIteration:
            break
    return None

#####################################

# def accelerate_list_generator(generator, max_attempts=1, max_time=np.inf):
#     while True:
#         start_time = time.time()
#         for i in range(max_attempts):
#             if max_time <= elapsed_time(start_time):
#                 break
#             sequence = next(generator)
#             if sequence:
#                 yield sequence
#                 break
