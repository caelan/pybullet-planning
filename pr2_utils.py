import math
import os
import random
import re

import numpy as np

from utils import multiply, get_link_pose, joint_from_name, link_from_name, set_joint_position, set_joint_positions, \
    get_joint_positions, get_min_limit, get_max_limit, quat_from_euler, read_pickle, set_pose, set_base_values, \
    get_pose, euler_from_quat, link_from_name, \
    unit_point, unit_quat, unit_pose, get_center_extent, joints_from_names

#####################################

TOP_HOLDING_LEFT_ARM = [0.67717021, -0.34313199, 1.2, -1.46688405, 1.24223229, -1.95442826, 2.22254125]
SIDE_HOLDING_LEFT_ARM = [0.39277395, 0.33330058, 0., -1.52238431, 2.72170996, -1.21946936, -2.98914779]
REST_LEFT_ARM = [2.13539289, 1.29629967, 3.74999698, -0.15000005, 10000., -0.10000004, 10000.]
WIDE_LEFT_ARM = [1.5806603449288885, -0.14239066980481405, 1.4484623937179126, -1.4851759349218694, 1.3911839347271555,
                 -1.6531320011389408, -2.978586584568441]
CENTER_LEFT_ARM = [-0.07133691252641006, -0.052973836083405494, 1.5741805775919033, -1.4481146328076862,
                   1.571782540186805, -1.4891468812835686, -9.413338322697955]
# WIDE_RIGHT_ARM = [-1.3175723551150083, -0.09536552225976803, -1.396727055561703, -1.4433371993320296, -1.5334243909312468, -1.7298129320065025, 6.230244924007009]

PR2_GROUPS = {
    'base': ['x', 'y', 'theta'],
    'torso': ['torso_lift_joint'],
    'head': ['head_pan_joint', 'head_tilt_joint'],
    'left_arm': ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
                 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint'],
    'right_arm': ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                  'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'],
    'left_gripper': ['l_gripper_l_finger_joint', 'l_gripper_r_finger_joint',
                     'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_tip_joint'],
    'right_gripper': ['r_gripper_l_finger_joint', 'r_gripper_r_finger_joint',
                      'r_gripper_l_finger_tip_joint', 'r_gripper_r_finger_tip_joint'],
}
PR2_TOOL_FRAMES = {
    'left_gripper': 'l_gripper_palm_link',  # l_gripper_palm_link | l_gripper_tool_frame
    'right_gripper': 'r_gripper_palm_link',  # r_gripper_palm_link | r_gripper_tool_frame
}

ARM_JOINT_NAMES = {'left': PR2_GROUPS['left_arm'], 'right': PR2_GROUPS['right_arm']}

ARM_LINK_NAMES = {
    'left': 'l_gripper_palm_link',  # l_gripper_tool_frame,
    'right': 'r_gripper_palm_link',
}

TORSO_JOINT_NAME = 'torso_lift_joint'
# LEFT_TOOL_NAME = 'l_gripper_tool_frame' # l_gripper_tool_joint | l_gripper_tool_frame

# TOOL_TFORM = [[0., 0., 1., 0.18],
#              [0., 1., 0., 0.],
#              [-1., 0., 0., 0.],
#              [0., 0., 0., 1.]]
TOOL_POSE = ([0.18, 0., 0.], [0., 0.70710678, 0., 0.70710678])
TOOL_DIRECTION = [0., 0., 1.]

PR2_DISABLED_COLLISIONS = [('base_link', 'fr_caster_l_wheel_link'), ('base_link', 'fr_caster_r_wheel_link'),
                           ('base_link', 'br_caster_l_wheel_link'), ('base_link', 'br_caster_r_wheel_link'),
                           ('base_link', 'bl_caster_l_wheel_link'), ('base_link', 'bl_caster_r_wheel_link'),
                           ('base_link', 'fl_caster_l_wheel_link'), ('base_link', 'fl_caster_r_wheel_link'),
                           ('base_link', 'projector_wg6802418_frame'), ('base_link', 'r_forearm_cam_optical_frame'),
                           ('torso_lift_link', 'projector_wg6802418_child_frame'),
                           ('torso_lift_link', 'l_shoulder_pan_link'),
                           ('projector_wg6802418_frame', 'laser_tilt_mount_link'),
                           ('r_forearm_cam_optical_frame', 'l_shoulder_lift_link'),
                           ('projector_wg6802418_frame', 'laser_tilt_link'),
                           ('r_forearm_cam_optical_frame', 'l_upper_arm_roll_link'),
                           ('r_shoulder_pan_link', 'r_elbow_flex_link'), ('l_upper_arm_link', 'l_wrist_flex_link'),
                           ('laser_tilt_link', 'r_shoulder_lift_link'), ('l_upper_arm_roll_link', 'l_elbow_flex_link'),
                           ('r_elbow_flex_link', 'r_wrist_flex_link'), ('l_wrist_flex_link', 'l_gripper_led_frame'),
                           ('r_elbow_flex_link', 'r_forearm_link'), ('l_wrist_flex_link', 'l_gripper_palm_link'),
                           ('r_wrist_roll_link', 'r_gripper_motor_screw_link'),
                           ('l_gripper_motor_accelerometer_link', 'l_gripper_r_finger_link'),
                           ('r_gripper_palm_link', 'r_gripper_motor_screw_link'),
                           ('l_gripper_tool_frame', 'l_gripper_r_finger_link'),
                           ('r_wrist_roll_link', 'r_gripper_l_finger_link'),
                           ('l_gripper_motor_accelerometer_link', 'l_gripper_r_finger_tip_link'),
                           ('torso_lift_link', 'r_torso_lift_side_plate_link')]


def rightarm_from_leftarm(config):
    # right_from_left = np.array([-1, 1, -1, 1, -1, 1, 1])
    right_from_left = np.array([-1, 1, -1, 1, -1, 1, -1])  # Drake
    return config * right_from_left


REST_RIGHT_ARM = rightarm_from_leftarm(REST_LEFT_ARM)
TOP_HOLDING_RIGHT_ARM = rightarm_from_leftarm(TOP_HOLDING_LEFT_ARM)


def arm_conf(arm, config):
    assert arm in ARM_JOINT_NAMES
    if arm == 'left':
        return config
    else:
        return rightarm_from_leftarm(config)


def get_carry_conf(arm, grasp_type):
    if grasp_type == 'top':
        return arm_conf(arm, TOP_HOLDING_LEFT_ARM)
    elif grasp_type == 'side':
        return arm_conf(arm, SIDE_HOLDING_LEFT_ARM)
    else:
        raise NotImplementedError()


def get_other_arm(arm):
    for other_arm in ARM_JOINT_NAMES:
        if other_arm != arm:
            return other_arm
    raise ValueError(arm)

def get_disabled_collisions(pr2):
    return [(link_from_name(pr2, name1), link_from_name(pr2, name2))
            for name1, name2 in PR2_DISABLED_COLLISIONS]

def load_disabled_collisions():
    # <link sid="link21" name="projector_wg6802418_child_frame">


    #assert(not use_pr2_drake)
    #disabled_collisions = load_disabled_collisions()
    #print([(get_link_name(pr2, link1), get_link_name(pr2, link2)) for link1, link2 in disabled_collisions])

    # pr2-beta-static.dae: link 0 = base_footprint
    # pybullet: link -1 = base_footprint
    dae_file = 'models/pr2_description/pr2-beta-static.dae'
    regex = r'<\s*ignore_link_pair\s+link0="kmodel1/link(\d+)"\s+link1="kmodel1/link(\d+)"\s*/>'
    disabled_collisions = []
    dae_file = open(dae_file).read()
    for link1, link2 in re.findall(regex, dae_file):
        disabled_collisions.append((int(link1) - 1, int(link2) - 1))
    return disabled_collisions

#####################################

# End-effectors

def get_arm_joints(robot, arm):
    assert arm in ARM_JOINT_NAMES
    return joints_from_names(robot, ARM_JOINT_NAMES[arm])


def get_arm_conf(robot, arm):
    return get_joint_positions(robot, get_arm_joints(robot, arm))


def set_arm_conf(robot, arm, conf):
    set_joint_positions(robot, get_arm_joints(robot, arm), conf)


def get_gripper_link(robot, arm):
    assert arm in ARM_LINK_NAMES
    return link_from_name(robot, ARM_LINK_NAMES[arm])


# def get_gripper_pose(robot):
#    # world_from_gripper * gripper_from_tool * tool_from_object = world_from_object
#    pose = multiply(get_link_pose(robot, link_from_name(robot, LEFT_ARM_LINK)), TOOL_POSE)
#    #pose = get_link_pose(robot, link_from_name(robot, LEFT_TOOL_NAME))
#    return pose

def open_arm(robot, arm):  # TODO: these are mirrored on the pr2
    gripper_name = '{}_gripper'.format(arm)
    for name in PR2_GROUPS[gripper_name]:
        joint = joint_from_name(robot, name)
        set_joint_position(robot, joint, get_max_limit(robot, joint))


def close_arm(robot, arm):
    gripper_name = '{}_gripper'.format(arm)
    for name in PR2_GROUPS[gripper_name]:
        joint = joint_from_name(robot, name)
        set_joint_position(robot, joint, get_min_limit(robot, joint))


# GRASP_LENGTH = 0.04
GRASP_LENGTH = 0.
MAX_GRASP_WIDTH = 0.07


def get_top_grasps(body, under=False, tool_pose=TOOL_POSE,
                   max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    pose = get_pose(body)
    set_pose(body, unit_pose())
    center, (w, l, h) = get_center_extent(body)
    reflect_z = (np.zeros(3), quat_from_euler([0, math.pi, 0]))
    translate = ([0, 0, h / 2 - grasp_length], unit_quat())
    grasps = []
    if w <= max_width:
        for i in range(1 + under):
            rotate_z = (unit_point(), quat_from_euler([0, 0, math.pi / 2 + i * math.pi]))
            grasps += [multiply(tool_pose, translate, rotate_z, reflect_z)]
    if l <= max_width:
        for i in range(1 + under):
            rotate_z = (unit_point(), quat_from_euler([0, 0, i * math.pi]))
            grasps += [multiply(tool_pose, translate, rotate_z, reflect_z)]
    set_pose(body, pose)
    return grasps


def get_side_grasps(body, under=False, limits=True, grasp_length=GRASP_LENGTH):
    pose = get_pose(body)
    set_pose(body, unit_pose())
    center, (w, l, h) = get_center_extent(body)
    grasps = []
    for j in range(1 + under):
        swap_xz = (unit_point(), quat_from_euler([0, -math.pi / 2 + j * math.pi, 0]))
        # if not limits or (w <= MAX_GRASP_WIDTH):
        if True:
            translate = ([0, 0, l / 2 - grasp_length], unit_quat())
            for i in range(2):
                rotate_z = (unit_point(), quat_from_euler([math.pi / 2 + i * math.pi, 0, 0]))
                grasps += [multiply(TOOL_POSE, translate, rotate_z, swap_xz)]  # , np.array([w])
        if True:
            # if not limits or (l <= MAX_GRASP_WIDTH):
            translate = ([0, 0, w / 2 - grasp_length], unit_quat())
            for i in range(2):
                rotate_z = (unit_point(), quat_from_euler([i * math.pi, 0, 0]))
                grasps += [multiply(TOOL_POSE, translate, rotate_z, swap_xz)]  # , np.array([l])
    set_pose(body, pose)
    return grasps


def get_x_presses(body, max_orientations=1):  # g_f_o
    pose = get_pose(body)
    set_pose(body, unit_pose())
    center, (w, l, h) = get_center_extent(body)
    press_poses = []
    for j in range(max_orientations):
        swap_xz = (unit_point(), quat_from_euler([0, -math.pi / 2 + j * math.pi, 0]))
        translate = ([0, 0, w / 2], unit_quat())
        press_poses += [multiply(TOOL_POSE, translate, swap_xz)]
    set_pose(body, pose)
    return press_poses


GET_GRASPS = {
    'top': get_top_grasps,
    'side': get_side_grasps,
    # 'press': get_x_presses,
}

#####################################

DATABASES_DIR = 'databases'
IR_FILENAME = '{}_{}_ir.pickle'


def load_inverse_reachability(arm, grasp_type):
    filename = IR_FILENAME.format(grasp_type, arm)
    path = os.path.join(DATABASES_DIR, filename)
    return read_pickle(path)['gripper_from_base']


def learned_pose_generator(robot, gripper_pose, arm, grasp_type):
    gripper_from_base_list = load_inverse_reachability(arm, grasp_type)
    random.shuffle(gripper_from_base_list)
    for gripper_from_base in gripper_from_base_list:
        base_point, base_quat = multiply(gripper_pose, gripper_from_base)
        x, y, _ = base_point
        _, _, theta = euler_from_quat(base_quat)
        base_values = (x, y, theta)
        set_base_values(robot, base_values)
        yield get_pose(robot)
