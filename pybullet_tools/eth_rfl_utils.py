
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
        arm_joints('right'): ['robot_joint_1', 'robot_joint_2', 'robot_joint_3', 'robot_joint_4', 'robot_joint_5', 'robot_joint_6']
        }
