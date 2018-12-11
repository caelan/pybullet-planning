from ...utils import matrix_from_quat, point_from_pose, quat_from_pose, quat_from_matrix, multiply, \
    get_link_pose, link_from_name, joints_from_names, get_joint_positions, get_joint_limits, joint_from_name, invert, \
    get_joint_position, violates_limits
from ...pr2_utils import PR2_GROUPS, PR2_TOOL_FRAMES, arm_from_arm

import random

IK_LINK = {
    'left': 'l_gripper_tool_frame',
    'right': 'r_gripper_tool_frame',
}

BASE_LINK = 'base_link'
TORSO_JOINT = 'torso_lift_joint'

UPPER_JOINT = {
    'left': 'l_upper_arm_roll_joint',
    'right': 'r_upper_arm_roll_joint',
}

# tool = l_gripper_tool_frame | r_gripper_tool_frame
# upper = l_upper_arm_roll_joint | r_upper_arm_roll_joint
# torso = torso_lift_joint

def get_base_pose(robot):
    return get_link_pose(robot, link_from_name(robot, BASE_LINK))

def get_torso_arm_joints(robot, arm):
    return joints_from_names(robot, PR2_GROUPS['torso'] + PR2_GROUPS[arm_from_arm(arm)])

#####################################

def forward_kinematics(arm, conf):
    from .ikLeft import leftFK
    from .ikRight import rightFK
    arm_fk = {
        'left': leftFK,
        'right': rightFK,
    }
    # base_link -> l_gripper_tool_frame | r_gripper_tool_frame
    assert len(conf) == 8
    fk_fn = arm_fk[arm]
    pose = fk_fn(list(conf))
    pos, rot = pose
    quat = quat_from_matrix(rot) # [X,Y,Z,W]
    return pos, quat
    # switch from [r, i, j, k] to [i, j, k, r]
    #quat = quat if quat.real >= 0 else -quat  # solves q and -q being same rotation

def get_tool_pose(robot, arm):
    # TODO: compute static transform from base_footprint -> base_link
    tool_pose = forward_kinematics(arm, get_joint_positions(robot, get_torso_arm_joints(robot, arm)))
    return multiply(get_base_pose(robot), tool_pose)

#####################################

def is_ik_compiled():
    try:
        from .ikLeft import leftIK
        from .ikRight import rightIK
        return True
    except ImportError:
        return False

def inverse_kinematics(arm, pose, torso, upper):
    from .ikLeft import leftIK
    from .ikRight import rightIK
    arm_ik = {
        'left': leftIK,
        'right': rightIK,
    }
    ik_fn = arm_ik[arm]
    pos = point_from_pose(pose)
    rot = matrix_from_quat(quat_from_pose(pose)).tolist()
    solutions = ik_fn(list(rot), list(pos), [torso, upper])
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

def get_ik_generator(robot, arm, world_pose, torso_limits=False, upper_limits=False):
    target_pose = multiply(invert(get_base_pose(robot)), world_pose)
    torso_limits = get_ik_limits(robot, joint_from_name(robot, TORSO_JOINT), torso_limits)
    upper_limits = get_ik_limits(robot, joint_from_name(robot, UPPER_JOINT[arm]), upper_limits)
    arm_joints = get_torso_arm_joints(robot, arm)
    while True:
        torso = random.uniform(*torso_limits)
        upper = random.uniform(*upper_limits)
        yield [q for q in inverse_kinematics(arm, target_pose, torso, upper)
               if not violates_limits(robot, arm_joints, q)]

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
