from pybullet_tools.kuka_kr6r900_ik.ikfast_kuka_kr6r900 import get_ik, get_fk

from pybullet_tools.utils import matrix_from_quat, point_from_pose, quat_from_pose, quat_from_matrix, Pose, multiply, elapsed_time, \
    get_link_pose, link_from_name, joints_from_names, get_joint_positions, get_joint_limits, joint_from_name, invert, \
    get_joint_position, violates_limits, INF, get_joints

import random

# the corresponding frame name can be found in
# models/kuka_kr6r900_description/framefab_kr6_r900_support/urdf/kuka_kr6_r900.urdf
KUKA_KR6R900_GROUPS = {
    'arm_joints': ['robot_joint_a1', 'robot_joint_a2', 'robot_joint_a3',
                   'robot_joint_a4', 'robot_joint_a5', 'robot_joint_a6'],
    'tool_link': 'robot_tool0',
    'ik_tool_link': 'robot_tool0'
}

BASE_LINK = 'robot_base_link'

def or_from_pyb_quat(quat):
    x, y, z, w = quat
    return [w, x, y, z]

def pyb_from_or_quat(quat):
    w, x, y, z = quat
    return [x, y, z, w]

def get_base_pose(robot):
    return get_link_pose(robot, link_from_name(robot, BASE_LINK))

#####################################

def forward_kinematics(conf):
    # TODO: this should be linked to ikfast's get numOfJoint function.
    print(conf)
    assert len(conf) == 6

    fk_fn = get_fk

    pose = fk_fn(list(conf))
    pos, rot = pose
    quat = quat_from_matrix(rot) # [X,Y,Z,W]
    return pos, quat
    # switch from [r, i, j, k] to [i, j, k, r]
    # quat = quat if quat.real >= 0 else -quat  # solves q and -q being same rotation

def get_tool_pose(robot):
    '''
    compute static transform from BASE_LINK -> IK_LINK
    :param robot:
    :param arm:
    :return:
    '''
    tool_pose = forward_kinematics(get_joint_positions(
        robot, joints_from_names(robot, KUKA_KR6R900_GROUPS['arm_joints'])))
    return multiply(get_base_pose(robot), tool_pose)

#####################################

def inverse_kinematics(pose):
    '''
    Compute IK. There is no free joint in this 6-dof robot.
    :param pose: end effector's pose
    :return:
    '''
    ik_fn = get_ik

    pos = point_from_pose(pose)
    rot = matrix_from_quat(quat_from_pose(pose)).tolist()
    solutions = ik_fn(list(rot), list(pos))
    if solutions is None:
        return []
    return solutions

#def sample_ik(robot, arm, target_pose, torso_limits, upper_limits):
#    arm_joints = get_arm_joints(robot, arm)
#    torso = random.uniform(*torso_limits)
#    upper = random.uniform(*upper_limits)
#    return [q for q in inverse_kinematics(arm, target_pose, torso, upper)
#           if not violates_limits(robot, arm_joints, q)]

def get_ik_generator(robot, ee_world_pose):
    target_ee_pose = multiply(invert(get_base_pose(robot)), ee_world_pose)

    # get joint limits
    arm_joints = joints_from_names(robot, KUKA_KR6R900_GROUPS['arm_joints'])
    while True:
        yield [q for q in inverse_kinematics(target_ee_pose)
               if not violates_limits(robot, arm_joints, q)]

def sample_tool_ik(robot, world_from_target, max_attempts=10, **kwargs):
    world_from_tool = get_link_pose(robot, link_from_name(robot, KUKA_KR6R900_GROUPS['tool_link']))
    world_from_ik = get_link_pose(robot, link_from_name(robot, KUKA_KR6R900_GROUPS['ik_tool_link']))

    # tool from the bare flange (6th axis)
    tool_from_ik = multiply(invert(world_from_tool), world_from_ik)
    ik_pose = multiply(world_from_target, tool_from_ik)

    generator = get_ik_generator(robot, ik_pose, **kwargs)
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
