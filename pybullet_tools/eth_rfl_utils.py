from .utils import joints_from_names, joint_from_name, link_from_name
from .kuka_primitives import GRASP_INFO, BodyGrasp

ARM_NAMES = {'left', 'right'}

# dict name composers
def arm_joints_from_arm(arm):
    assert (arm in ARM_NAMES)
    return '{}_arm'.format(arm)

def torso_from_arm(arm):
    assert (arm in ARM_NAMES)
    return '{}_torso'.format(arm)

ETH_RFL_GROUPS = {
        'base': ['x', 'y'],
        torso_from_arm('left'): ['l_gantry_z_joint'],
        torso_from_arm('right'): ['r_gantry_z_joint'],
        arm_joints_from_arm('left'): ['l_robot_joint_1', 'l_robot_joint_2', 'l_robot_joint_3', 'l_robot_joint_4', 'l_robot_joint_5', 'l_robot_joint_6'],
        arm_joints_from_arm('right'): ['r_robot_joint_1', 'r_robot_joint_2', 'r_robot_joint_3', 'r_robot_joint_4', 'r_robot_joint_5', 'r_robot_joint_6'],
        }

ETH_RFL_TOOL_FRAMES = {
    'left': 'l_eef_tcp_frame',
    'right': 'r_eef_tcp_frame',
}

# joint id in pybullet
def get_torso_arm_joints(robot, arm):
    return joints_from_names(robot, get_torso_arm_joint_names(arm))

def get_torso_joint(robot, arm):
    return joint_from_name(robot, *ETH_RFL_GROUPS[torso_from_arm(arm)])

# frame name (udrf)
def get_torso_arm_joint_names(arm):
    return ETH_RFL_GROUPS[torso_from_arm(arm)] + ETH_RFL_GROUPS[arm_joints_from_arm(arm)]

def get_tool_frame(arm):
    return ETH_RFL_TOOL_FRAMES[arm]

def get_grasp_gen(robot, arm, grasp_name):
    grasp_info = GRASP_INFO[grasp_name]
    end_effector_link = link_from_name(robot, get_tool_frame(arm))
    def gen(body):
        grasp_poses = grasp_info.get_grasps(body)
        for grasp_pose in grasp_poses:
            body_grasp = BodyGrasp(body, grasp_pose, grasp_info.approach_pose,
                                   robot, end_effector_link)
            yield (body_grasp,)
    return gen
