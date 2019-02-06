from .utils import joints_from_names

ARM_NAMES = {'left', 'right'}

def arm_joints(arm_name):
    assert (arm_name in ARM_NAMES)
    return '{}_arm'.format(arm_name)

def torso_from_arm(arm_name):
    assert (arm_name in ARM_NAMES)
    return '{}_torso'.format(arm_name)

ETH_RFL_GROUPS = {
        'base': ['x', 'y'],
        torso_from_arm('right'): ['gantry_z_joint'],
        arm_joints('right'): ['r_robot_joint_1', 'r_robot_joint_2', 'r_robot_joint_3', 'r_robot_joint_4', 'r_robot_joint_5', 'r_robot_joint_6']
        }

def get_torso_arm_joints(robot, arm):
    """get joint name of an arm

    :param robot:
    :param arm: 'left' or 'right'
    :return: a int list of joint names for pybullet engine
    """
    return joints_from_names(robot, ETH_RFL_GROUPS[torso_from_arm(arm)] + ETH_RFL_GROUPS[arm_joints(arm)])
