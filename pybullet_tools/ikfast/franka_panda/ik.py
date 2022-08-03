from ..utils import IKFastInfo
from ..utils import get_ik_limits, compute_forward_kinematics, compute_inverse_kinematics, select_solution, \
    USE_ALL, USE_CURRENT
from ...panda_utils import PANDA_TOOL_FRAMES, get_gripper_link, get_arm_joints, side_from_arm
from ...utils import multiply, get_link_pose, link_from_name, get_joint_positions, \
    joint_from_name, invert, get_custom_limits, all_between, sub_inverse_kinematics, set_joint_positions, \
    get_joint_positions, pairwise_collision
from ..ikfast import * # For legacy purposes

# TODO: deprecate this file
#FRANKA_URDF = "models/franka_description/robots/panda_arm.urdf"
#FRANKA_URDF = "models/franka_description/robots/hand.urdf"
IK_FRAME = {
    'left': 'l_panda_grasptarget',
    'right': 'r_panda_grasptarget',
}
FRANKA_URDF = "models/franka_description/robots/panda_arm_hand.urdf"

PANDA_INFO = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='panda_link0',
                        ee_link='panda_link8', free_joints=['panda_joint7'])

PANDA_LEFT_INFO = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='l_panda_link0',
                        ee_link='l_panda_link8', free_joints=['l_panda_joint7'])

PANDA_RIGHT_INFO = IKFastInfo(module_name='franka_panda.ikfast_panda_arm', base_link='r_panda_link0',
                        ee_link='r_panda_grasptarget', free_joints=['r_panda_joint7'])

info = {'left': PANDA_LEFT_INFO, 'right': PANDA_RIGHT_INFO}

def get_tool_from_ik(robot, arm):
    # TODO: change PR2_TOOL_FRAMES[arm] to be IK_LINK[arm]
    world_from_tool = get_link_pose(robot, link_from_name(robot, PANDA_TOOL_FRAMES[arm]))
    world_from_ik = get_link_pose(robot, link_from_name(robot, IK_FRAME[arm]))
    return multiply(invert(world_from_tool), world_from_ik)

def get_ik_generator(robot, arm, gripper_link, gripper_pose, max_attempts=25, max_time=1.3):
  return ikfast_inverse_kinematics(robot, info[arm], gripper_link, gripper_pose, max_attempts=25, max_time=1.3)

def get_joint_distances(current_config, new_config):
    d = 0
    n = len(new_config)
    for i in range(len(new_config)):
        d += ((new_config[i] - current_config[i])**2/n)
    return d

def sample_tool_ik(robot, arm, tool_pose, nearby_conf=USE_CURRENT, max_attempts=25, custom_limits={}, **kwargs):
    ik_pose = multiply(tool_pose, get_tool_from_ik(robot, arm))
    generator = get_ik_generator(robot, arm, link_from_name(robot, PANDA_TOOL_FRAMES[arm]), ik_pose, **kwargs)
    arm_joints = get_arm_joints(robot, arm)
    current_conf = get_joint_positions(robot, arm_joints)
    for _ in range(max_attempts):
        try:
            solutions = next(generator)
            # TODO: sort by distance from the current solution when attempting?
            if solutions:
                # distances = [get_joint_distances(current_conf, new_config) for new_config in solutions]
                return solutions
        except StopIteration:
            break
    return None

def bi_panda_inverse_kinematics(robot, arm, gripper_link, gripper_pose, max_attempts=25, max_time=1.3, custom_limits={}, obstacles=[]):
    arm_link = get_gripper_link(robot, arm)
    arm_joints = get_arm_joints(robot, arm)
    if is_ik_compiled(info[arm]):
        ik_joints = get_arm_joints(robot, arm)
        arm_conf = sample_tool_ik(robot, arm, gripper_pose, custom_limits=custom_limits)
        if arm_conf is None:
            print("arm conf is none")
            return None
        set_joint_positions(robot, ik_joints, arm_conf)
    else:
        arm_conf = sub_inverse_kinematics(robot, arm_joints[0], arm_link, gripper_pose, custom_limits=custom_limits)
        if arm_conf is None:
            return None
    if any(pairwise_collision(robot, b) for b in obstacles):
        return None
    return get_joint_positions(robot, arm_joints)