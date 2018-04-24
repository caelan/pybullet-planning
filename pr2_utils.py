import math
import os
import random
import re

import numpy as np

from utils import multiply, get_link_pose, joint_from_name, link_from_name, set_joint_position, set_joint_positions, \
    get_joint_positions, get_min_limit, get_max_limit, quat_from_euler, read_pickle, set_pose, set_base_values, \
    get_pose, euler_from_quat, link_from_name, has_link, \
    unit_point, unit_quat, unit_pose, get_center_extent, joints_from_names
from pr2_self_collision import PR2_ADJACENT_LINKS, PR2_NON_ADJACENT_LINKS

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

PR2_DISABLED_COLLISIONS = [('base_link', 'br_caster_l_wheel_link'), ('base_link', 'br_caster_r_wheel_link'),
                           ('base_link', 'fr_caster_l_wheel_link'), ('base_link', 'fr_caster_r_wheel_link'),
                           ('base_link', 'fl_caster_l_wheel_link'), ('base_link', 'fl_caster_r_wheel_link'),
                           ('base_link', 'bl_caster_l_wheel_link'), ('base_link', 'bl_caster_r_wheel_link'),
                           ('base_link', 'l_shoulder_pan_link'), ('base_link', 'r_shoulder_pan_link'),
                           ('torso_lift_link', 'l_shoulder_lift_link'), ('torso_lift_link', 'r_shoulder_lift_link'),
                           ('l_shoulder_pan_link', 'l_upper_arm_roll_link'),
                           ('r_shoulder_pan_link', 'r_upper_arm_roll_link'),
                           ('l_shoulder_pan_link', 'l_upper_arm_link'), ('r_shoulder_pan_link', 'r_upper_arm_link'),
                           ('l_elbow_flex_link', 'l_forearm_link'), ('r_elbow_flex_link', 'r_forearm_link'),
                           ('l_upper_arm_link', 'l_forearm_roll_link'), ('r_upper_arm_link', 'r_forearm_roll_link'),
                           ('l_forearm_link', 'l_gripper_palm_link'), ('r_forearm_link', 'r_gripper_palm_link'),
                           ('l_forearm_link', 'l_wrist_roll_link'), ('r_forearm_link', 'r_wrist_roll_link'),
                           ('l_gripper_l_finger_link', 'l_gripper_r_finger_link'),
                           ('r_gripper_l_finger_link', 'r_gripper_r_finger_link'),
                           ('l_gripper_l_finger_tip_link', 'l_gripper_r_finger_link'),
                           ('r_gripper_l_finger_tip_link', 'r_gripper_r_finger_link'),
                           ('l_gripper_l_finger_link', 'l_gripper_r_finger_tip_link'),
                           ('r_gripper_l_finger_link', 'r_gripper_r_finger_tip_link'),
                           ('torso_lift_link', 'head_tilt_link')
] + [
    #('l_upper_arm_link', 'l_shoulder_lift_link'),
    #('r_upper_arm_link', 'r_shoulder_lift_link'),
]

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
    #disabled_names = PR2_ADJACENT_LINKS
    #disabled_names = PR2_DISABLED_COLLISIONS
    disabled_names = NEVER_COLLISIONS
    #disabled_names = PR2_DISABLED_COLLISIONS + NEVER_COLLISIONS
    return [(link_from_name(pr2, name1), link_from_name(pr2, name2))
            for name1, name2 in disabled_names if has_link(pr2, name1) and has_link(pr2, name2)]


def load_dae_collisions():
    # pr2-beta-static.dae: link 0 = base_footprint
    # pybullet: link -1 = base_footprint
    dae_file = 'models/pr2_description/pr2-beta-static.dae'
    dae_string = open(dae_file).read()
    link_regex = r'<\s*link\s+sid="(\w+)"\s+name="(\w+)"\s*>'
    link_mapping = dict(re.findall(link_regex, dae_string))
    ignore_regex = r'<\s*ignore_link_pair\s+link0="kmodel1/(\w+)"\s+link1="kmodel1/(\w+)"\s*/>'
    disabled_collisions = []
    for link1, link2 in re.findall(ignore_regex, dae_string):
        disabled_collisions.append((link_mapping[link1], link_mapping[link2]))
    return disabled_collisions


def load_srdf_collisions():
    srdf_file = 'models/pr2_description/pr2.srdf'
    srdf_string = open(srdf_file).read()
    regex = r'<\s*disable_collisions\s+link1="(\w+)"\s+link2="(\w+)"\s+reason="(\w+)"\s*/>'
    disabled_collisions = []
    for link1, link2, reason in re.findall(regex, srdf_string):
        if reason == 'Never':
            disabled_collisions.append((link1, link2))
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

def get_group_conf(robot, group):
    return get_joint_positions(robot, joints_from_names(robot, PR2_GROUPS[group]))

def set_group_conf(robot, group, positions):
    set_joint_positions(robot, joints_from_names(robot, PR2_GROUPS[group]), positions)

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


#GRASP_LENGTH = 0.04
GRASP_LENGTH = 0.
#GRASP_LENGTH = -0.01
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


NEVER_COLLISIONS = [('base_bellow_link', 'base_footprint'), ('base_bellow_link', 'bl_caster_l_wheel_link'),
                    ('base_bellow_link', 'bl_caster_r_wheel_link'), ('base_bellow_link', 'bl_caster_rotation_link'),
                    ('base_bellow_link', 'br_caster_l_wheel_link'), ('base_bellow_link', 'br_caster_r_wheel_link'),
                    ('base_bellow_link', 'br_caster_rotation_link'), ('base_bellow_link', 'double_stereo_link'),
                    ('base_bellow_link', 'fl_caster_l_wheel_link'), ('base_bellow_link', 'fl_caster_r_wheel_link'),
                    ('base_bellow_link', 'fl_caster_rotation_link'), ('base_bellow_link', 'fr_caster_l_wheel_link'),
                    ('base_bellow_link', 'fr_caster_r_wheel_link'), ('base_bellow_link', 'fr_caster_rotation_link'),
                    ('base_bellow_link', 'head_pan_link'), ('base_bellow_link', 'head_plate_frame'),
                    ('base_bellow_link', 'head_tilt_link'), ('base_bellow_link', 'l_elbow_flex_link'),
                    ('base_bellow_link', 'l_forearm_link'), ('base_bellow_link', 'l_forearm_roll_link'),
                    ('base_bellow_link', 'l_gripper_motor_accelerometer_link'),
                    ('base_bellow_link', 'l_shoulder_lift_link'), ('base_bellow_link', 'l_shoulder_pan_link'),
                    ('base_bellow_link', 'l_upper_arm_link'), ('base_bellow_link', 'l_upper_arm_roll_link'),
                    ('base_bellow_link', 'l_wrist_roll_link'), ('base_bellow_link', 'laser_tilt_mount_link'),
                    ('base_bellow_link', 'r_elbow_flex_link'), ('base_bellow_link', 'r_forearm_link'),
                    ('base_bellow_link', 'r_forearm_roll_link'),
                    ('base_bellow_link', 'r_gripper_motor_accelerometer_link'),
                    ('base_bellow_link', 'r_shoulder_lift_link'), ('base_bellow_link', 'r_shoulder_pan_link'),
                    ('base_bellow_link', 'r_upper_arm_link'), ('base_bellow_link', 'r_upper_arm_roll_link'),
                    ('base_bellow_link', 'r_wrist_flex_link'), ('base_bellow_link', 'r_wrist_roll_link'),
                    ('base_bellow_link', 'sensor_mount_link'), ('base_footprint', 'bl_caster_l_wheel_link'),
                    ('base_footprint', 'bl_caster_r_wheel_link'), ('base_footprint', 'bl_caster_rotation_link'),
                    ('base_footprint', 'br_caster_l_wheel_link'), ('base_footprint', 'br_caster_r_wheel_link'),
                    ('base_footprint', 'br_caster_rotation_link'), ('base_footprint', 'double_stereo_link'),
                    ('base_footprint', 'fl_caster_l_wheel_link'), ('base_footprint', 'fl_caster_r_wheel_link'),
                    ('base_footprint', 'fl_caster_rotation_link'), ('base_footprint', 'fr_caster_l_wheel_link'),
                    ('base_footprint', 'fr_caster_r_wheel_link'), ('base_footprint', 'fr_caster_rotation_link'),
                    ('base_footprint', 'head_pan_link'), ('base_footprint', 'head_plate_frame'),
                    ('base_footprint', 'head_tilt_link'), ('base_footprint', 'l_elbow_flex_link'),
                    ('base_footprint', 'l_forearm_link'), ('base_footprint', 'l_forearm_roll_link'),
                    ('base_footprint', 'l_gripper_l_finger_link'), ('base_footprint', 'l_gripper_l_finger_tip_link'),
                    ('base_footprint', 'l_gripper_motor_accelerometer_link'), ('base_footprint', 'l_gripper_palm_link'),
                    ('base_footprint', 'l_gripper_r_finger_link'), ('base_footprint', 'l_gripper_r_finger_tip_link'),
                    ('base_footprint', 'l_shoulder_lift_link'), ('base_footprint', 'l_shoulder_pan_link'),
                    ('base_footprint', 'l_upper_arm_link'), ('base_footprint', 'l_upper_arm_roll_link'),
                    ('base_footprint', 'l_wrist_flex_link'), ('base_footprint', 'l_wrist_roll_link'),
                    ('base_footprint', 'laser_tilt_mount_link'), ('base_footprint', 'r_elbow_flex_link'),
                    ('base_footprint', 'r_forearm_link'), ('base_footprint', 'r_forearm_roll_link'),
                    ('base_footprint', 'r_gripper_l_finger_link'), ('base_footprint', 'r_gripper_l_finger_tip_link'),
                    ('base_footprint', 'r_gripper_motor_accelerometer_link'), ('base_footprint', 'r_gripper_palm_link'),
                    ('base_footprint', 'r_gripper_r_finger_link'), ('base_footprint', 'r_gripper_r_finger_tip_link'),
                    ('base_footprint', 'r_shoulder_lift_link'), ('base_footprint', 'r_shoulder_pan_link'),
                    ('base_footprint', 'r_upper_arm_link'), ('base_footprint', 'r_upper_arm_roll_link'),
                    ('base_footprint', 'r_wrist_flex_link'), ('base_footprint', 'r_wrist_roll_link'),
                    ('base_footprint', 'sensor_mount_link'), ('base_footprint', 'torso_lift_link'),
                    ('base_link', 'double_stereo_link'), ('base_link', 'head_pan_link'),
                    ('base_link', 'head_plate_frame'), ('base_link', 'head_tilt_link'),
                    ('base_link', 'l_shoulder_lift_link'), ('base_link', 'l_shoulder_pan_link'),
                    ('base_link', 'l_upper_arm_link'), ('base_link', 'l_upper_arm_roll_link'),
                    ('base_link', 'laser_tilt_mount_link'), ('base_link', 'r_shoulder_lift_link'),
                    ('base_link', 'r_shoulder_pan_link'), ('base_link', 'r_upper_arm_link'),
                    ('base_link', 'r_upper_arm_roll_link'), ('base_link', 'sensor_mount_link'),
                    ('bl_caster_l_wheel_link', 'bl_caster_r_wheel_link'),
                    ('bl_caster_l_wheel_link', 'br_caster_l_wheel_link'),
                    ('bl_caster_l_wheel_link', 'br_caster_r_wheel_link'),
                    ('bl_caster_l_wheel_link', 'br_caster_rotation_link'),
                    ('bl_caster_l_wheel_link', 'double_stereo_link'),
                    ('bl_caster_l_wheel_link', 'fl_caster_l_wheel_link'),
                    ('bl_caster_l_wheel_link', 'fl_caster_r_wheel_link'),
                    ('bl_caster_l_wheel_link', 'fl_caster_rotation_link'),
                    ('bl_caster_l_wheel_link', 'fr_caster_l_wheel_link'),
                    ('bl_caster_l_wheel_link', 'fr_caster_r_wheel_link'),
                    ('bl_caster_l_wheel_link', 'fr_caster_rotation_link'), ('bl_caster_l_wheel_link', 'head_pan_link'),
                    ('bl_caster_l_wheel_link', 'head_plate_frame'), ('bl_caster_l_wheel_link', 'head_tilt_link'),
                    ('bl_caster_l_wheel_link', 'l_elbow_flex_link'), ('bl_caster_l_wheel_link', 'l_forearm_link'),
                    ('bl_caster_l_wheel_link', 'l_forearm_roll_link'),
                    ('bl_caster_l_wheel_link', 'l_gripper_motor_accelerometer_link'),
                    ('bl_caster_l_wheel_link', 'l_shoulder_lift_link'),
                    ('bl_caster_l_wheel_link', 'l_shoulder_pan_link'), ('bl_caster_l_wheel_link', 'l_upper_arm_link'),
                    ('bl_caster_l_wheel_link', 'l_upper_arm_roll_link'),
                    ('bl_caster_l_wheel_link', 'l_wrist_roll_link'),
                    ('bl_caster_l_wheel_link', 'laser_tilt_mount_link'),
                    ('bl_caster_l_wheel_link', 'r_elbow_flex_link'), ('bl_caster_l_wheel_link', 'r_forearm_link'),
                    ('bl_caster_l_wheel_link', 'r_forearm_roll_link'),
                    ('bl_caster_l_wheel_link', 'r_gripper_l_finger_link'),
                    ('bl_caster_l_wheel_link', 'r_gripper_l_finger_tip_link'),
                    ('bl_caster_l_wheel_link', 'r_gripper_motor_accelerometer_link'),
                    ('bl_caster_l_wheel_link', 'r_gripper_palm_link'),
                    ('bl_caster_l_wheel_link', 'r_gripper_r_finger_link'),
                    ('bl_caster_l_wheel_link', 'r_gripper_r_finger_tip_link'),
                    ('bl_caster_l_wheel_link', 'r_shoulder_lift_link'),
                    ('bl_caster_l_wheel_link', 'r_shoulder_pan_link'), ('bl_caster_l_wheel_link', 'r_upper_arm_link'),
                    ('bl_caster_l_wheel_link', 'r_upper_arm_roll_link'),
                    ('bl_caster_l_wheel_link', 'r_wrist_flex_link'), ('bl_caster_l_wheel_link', 'r_wrist_roll_link'),
                    ('bl_caster_l_wheel_link', 'sensor_mount_link'), ('bl_caster_l_wheel_link', 'torso_lift_link'),
                    ('bl_caster_r_wheel_link', 'br_caster_l_wheel_link'),
                    ('bl_caster_r_wheel_link', 'br_caster_r_wheel_link'),
                    ('bl_caster_r_wheel_link', 'br_caster_rotation_link'),
                    ('bl_caster_r_wheel_link', 'double_stereo_link'),
                    ('bl_caster_r_wheel_link', 'fl_caster_l_wheel_link'),
                    ('bl_caster_r_wheel_link', 'fl_caster_r_wheel_link'),
                    ('bl_caster_r_wheel_link', 'fl_caster_rotation_link'),
                    ('bl_caster_r_wheel_link', 'fr_caster_l_wheel_link'),
                    ('bl_caster_r_wheel_link', 'fr_caster_r_wheel_link'),
                    ('bl_caster_r_wheel_link', 'fr_caster_rotation_link'), ('bl_caster_r_wheel_link', 'head_pan_link'),
                    ('bl_caster_r_wheel_link', 'head_plate_frame'), ('bl_caster_r_wheel_link', 'head_tilt_link'),
                    ('bl_caster_r_wheel_link', 'l_elbow_flex_link'), ('bl_caster_r_wheel_link', 'l_forearm_roll_link'),
                    ('bl_caster_r_wheel_link', 'l_gripper_motor_accelerometer_link'),
                    ('bl_caster_r_wheel_link', 'l_shoulder_lift_link'),
                    ('bl_caster_r_wheel_link', 'l_shoulder_pan_link'), ('bl_caster_r_wheel_link', 'l_upper_arm_link'),
                    ('bl_caster_r_wheel_link', 'l_upper_arm_roll_link'),
                    ('bl_caster_r_wheel_link', 'laser_tilt_mount_link'),
                    ('bl_caster_r_wheel_link', 'r_elbow_flex_link'), ('bl_caster_r_wheel_link', 'r_forearm_link'),
                    ('bl_caster_r_wheel_link', 'r_forearm_roll_link'),
                    ('bl_caster_r_wheel_link', 'r_gripper_l_finger_link'),
                    ('bl_caster_r_wheel_link', 'r_gripper_l_finger_tip_link'),
                    ('bl_caster_r_wheel_link', 'r_gripper_motor_accelerometer_link'),
                    ('bl_caster_r_wheel_link', 'r_gripper_palm_link'),
                    ('bl_caster_r_wheel_link', 'r_gripper_r_finger_link'),
                    ('bl_caster_r_wheel_link', 'r_gripper_r_finger_tip_link'),
                    ('bl_caster_r_wheel_link', 'r_shoulder_lift_link'),
                    ('bl_caster_r_wheel_link', 'r_shoulder_pan_link'), ('bl_caster_r_wheel_link', 'r_upper_arm_link'),
                    ('bl_caster_r_wheel_link', 'r_upper_arm_roll_link'),
                    ('bl_caster_r_wheel_link', 'r_wrist_flex_link'), ('bl_caster_r_wheel_link', 'r_wrist_roll_link'),
                    ('bl_caster_r_wheel_link', 'sensor_mount_link'), ('bl_caster_r_wheel_link', 'torso_lift_link'),
                    ('bl_caster_rotation_link', 'br_caster_l_wheel_link'),
                    ('bl_caster_rotation_link', 'br_caster_r_wheel_link'),
                    ('bl_caster_rotation_link', 'br_caster_rotation_link'),
                    ('bl_caster_rotation_link', 'double_stereo_link'),
                    ('bl_caster_rotation_link', 'fl_caster_l_wheel_link'),
                    ('bl_caster_rotation_link', 'fl_caster_r_wheel_link'),
                    ('bl_caster_rotation_link', 'fl_caster_rotation_link'),
                    ('bl_caster_rotation_link', 'fr_caster_l_wheel_link'),
                    ('bl_caster_rotation_link', 'fr_caster_r_wheel_link'),
                    ('bl_caster_rotation_link', 'fr_caster_rotation_link'),
                    ('bl_caster_rotation_link', 'head_pan_link'), ('bl_caster_rotation_link', 'head_plate_frame'),
                    ('bl_caster_rotation_link', 'head_tilt_link'), ('bl_caster_rotation_link', 'l_elbow_flex_link'),
                    ('bl_caster_rotation_link', 'l_forearm_roll_link'),
                    ('bl_caster_rotation_link', 'l_shoulder_lift_link'),
                    ('bl_caster_rotation_link', 'l_shoulder_pan_link'), ('bl_caster_rotation_link', 'l_upper_arm_link'),
                    ('bl_caster_rotation_link', 'l_upper_arm_roll_link'),
                    ('bl_caster_rotation_link', 'laser_tilt_mount_link'),
                    ('bl_caster_rotation_link', 'r_elbow_flex_link'), ('bl_caster_rotation_link', 'r_forearm_link'),
                    ('bl_caster_rotation_link', 'r_forearm_roll_link'),
                    ('bl_caster_rotation_link', 'r_gripper_l_finger_link'),
                    ('bl_caster_rotation_link', 'r_gripper_l_finger_tip_link'),
                    ('bl_caster_rotation_link', 'r_gripper_motor_accelerometer_link'),
                    ('bl_caster_rotation_link', 'r_gripper_palm_link'),
                    ('bl_caster_rotation_link', 'r_gripper_r_finger_link'),
                    ('bl_caster_rotation_link', 'r_shoulder_lift_link'),
                    ('bl_caster_rotation_link', 'r_shoulder_pan_link'), ('bl_caster_rotation_link', 'r_upper_arm_link'),
                    ('bl_caster_rotation_link', 'r_upper_arm_roll_link'),
                    ('bl_caster_rotation_link', 'r_wrist_flex_link'), ('bl_caster_rotation_link', 'r_wrist_roll_link'),
                    ('bl_caster_rotation_link', 'sensor_mount_link'), ('bl_caster_rotation_link', 'torso_lift_link'),
                    ('br_caster_l_wheel_link', 'br_caster_r_wheel_link'),
                    ('br_caster_l_wheel_link', 'double_stereo_link'),
                    ('br_caster_l_wheel_link', 'fl_caster_l_wheel_link'),
                    ('br_caster_l_wheel_link', 'fl_caster_r_wheel_link'),
                    ('br_caster_l_wheel_link', 'fl_caster_rotation_link'),
                    ('br_caster_l_wheel_link', 'fr_caster_l_wheel_link'),
                    ('br_caster_l_wheel_link', 'fr_caster_r_wheel_link'),
                    ('br_caster_l_wheel_link', 'fr_caster_rotation_link'), ('br_caster_l_wheel_link', 'head_pan_link'),
                    ('br_caster_l_wheel_link', 'head_plate_frame'), ('br_caster_l_wheel_link', 'head_tilt_link'),
                    ('br_caster_l_wheel_link', 'l_elbow_flex_link'), ('br_caster_l_wheel_link', 'l_forearm_link'),
                    ('br_caster_l_wheel_link', 'l_forearm_roll_link'),
                    ('br_caster_l_wheel_link', 'l_gripper_l_finger_link'),
                    ('br_caster_l_wheel_link', 'l_gripper_l_finger_tip_link'),
                    ('br_caster_l_wheel_link', 'l_gripper_motor_accelerometer_link'),
                    ('br_caster_l_wheel_link', 'l_gripper_palm_link'),
                    ('br_caster_l_wheel_link', 'l_gripper_r_finger_link'),
                    ('br_caster_l_wheel_link', 'l_gripper_r_finger_tip_link'),
                    ('br_caster_l_wheel_link', 'l_shoulder_lift_link'),
                    ('br_caster_l_wheel_link', 'l_shoulder_pan_link'), ('br_caster_l_wheel_link', 'l_upper_arm_link'),
                    ('br_caster_l_wheel_link', 'l_upper_arm_roll_link'),
                    ('br_caster_l_wheel_link', 'l_wrist_flex_link'), ('br_caster_l_wheel_link', 'l_wrist_roll_link'),
                    ('br_caster_l_wheel_link', 'laser_tilt_mount_link'),
                    ('br_caster_l_wheel_link', 'r_elbow_flex_link'), ('br_caster_l_wheel_link', 'r_forearm_roll_link'),
                    ('br_caster_l_wheel_link', 'r_gripper_motor_accelerometer_link'),
                    ('br_caster_l_wheel_link', 'r_shoulder_lift_link'),
                    ('br_caster_l_wheel_link', 'r_shoulder_pan_link'), ('br_caster_l_wheel_link', 'r_upper_arm_link'),
                    ('br_caster_l_wheel_link', 'r_upper_arm_roll_link'),
                    ('br_caster_l_wheel_link', 'sensor_mount_link'), ('br_caster_l_wheel_link', 'torso_lift_link'),
                    ('br_caster_r_wheel_link', 'double_stereo_link'),
                    ('br_caster_r_wheel_link', 'fl_caster_l_wheel_link'),
                    ('br_caster_r_wheel_link', 'fl_caster_r_wheel_link'),
                    ('br_caster_r_wheel_link', 'fl_caster_rotation_link'),
                    ('br_caster_r_wheel_link', 'fr_caster_l_wheel_link'),
                    ('br_caster_r_wheel_link', 'fr_caster_r_wheel_link'),
                    ('br_caster_r_wheel_link', 'fr_caster_rotation_link'), ('br_caster_r_wheel_link', 'head_pan_link'),
                    ('br_caster_r_wheel_link', 'head_plate_frame'), ('br_caster_r_wheel_link', 'head_tilt_link'),
                    ('br_caster_r_wheel_link', 'l_elbow_flex_link'), ('br_caster_r_wheel_link', 'l_forearm_link'),
                    ('br_caster_r_wheel_link', 'l_forearm_roll_link'),
                    ('br_caster_r_wheel_link', 'l_gripper_l_finger_link'),
                    ('br_caster_r_wheel_link', 'l_gripper_l_finger_tip_link'),
                    ('br_caster_r_wheel_link', 'l_gripper_motor_accelerometer_link'),
                    ('br_caster_r_wheel_link', 'l_gripper_palm_link'),
                    ('br_caster_r_wheel_link', 'l_gripper_r_finger_link'),
                    ('br_caster_r_wheel_link', 'l_gripper_r_finger_tip_link'),
                    ('br_caster_r_wheel_link', 'l_shoulder_lift_link'),
                    ('br_caster_r_wheel_link', 'l_shoulder_pan_link'), ('br_caster_r_wheel_link', 'l_upper_arm_link'),
                    ('br_caster_r_wheel_link', 'l_upper_arm_roll_link'),
                    ('br_caster_r_wheel_link', 'l_wrist_flex_link'), ('br_caster_r_wheel_link', 'l_wrist_roll_link'),
                    ('br_caster_r_wheel_link', 'laser_tilt_mount_link'),
                    ('br_caster_r_wheel_link', 'r_elbow_flex_link'), ('br_caster_r_wheel_link', 'r_forearm_roll_link'),
                    ('br_caster_r_wheel_link', 'r_gripper_motor_accelerometer_link'),
                    ('br_caster_r_wheel_link', 'r_shoulder_lift_link'),
                    ('br_caster_r_wheel_link', 'r_shoulder_pan_link'), ('br_caster_r_wheel_link', 'r_upper_arm_link'),
                    ('br_caster_r_wheel_link', 'r_upper_arm_roll_link'),
                    ('br_caster_r_wheel_link', 'sensor_mount_link'), ('br_caster_r_wheel_link', 'torso_lift_link'),
                    ('br_caster_rotation_link', 'double_stereo_link'),
                    ('br_caster_rotation_link', 'fl_caster_l_wheel_link'),
                    ('br_caster_rotation_link', 'fl_caster_r_wheel_link'),
                    ('br_caster_rotation_link', 'fl_caster_rotation_link'),
                    ('br_caster_rotation_link', 'fr_caster_l_wheel_link'),
                    ('br_caster_rotation_link', 'fr_caster_r_wheel_link'),
                    ('br_caster_rotation_link', 'fr_caster_rotation_link'),
                    ('br_caster_rotation_link', 'head_pan_link'), ('br_caster_rotation_link', 'head_plate_frame'),
                    ('br_caster_rotation_link', 'head_tilt_link'), ('br_caster_rotation_link', 'l_elbow_flex_link'),
                    ('br_caster_rotation_link', 'l_forearm_link'), ('br_caster_rotation_link', 'l_forearm_roll_link'),
                    ('br_caster_rotation_link', 'l_gripper_motor_accelerometer_link'),
                    ('br_caster_rotation_link', 'l_gripper_palm_link'),
                    ('br_caster_rotation_link', 'l_gripper_r_finger_link'),
                    ('br_caster_rotation_link', 'l_gripper_r_finger_tip_link'),
                    ('br_caster_rotation_link', 'l_shoulder_lift_link'),
                    ('br_caster_rotation_link', 'l_shoulder_pan_link'), ('br_caster_rotation_link', 'l_upper_arm_link'),
                    ('br_caster_rotation_link', 'l_upper_arm_roll_link'),
                    ('br_caster_rotation_link', 'l_wrist_flex_link'), ('br_caster_rotation_link', 'l_wrist_roll_link'),
                    ('br_caster_rotation_link', 'laser_tilt_mount_link'),
                    ('br_caster_rotation_link', 'r_elbow_flex_link'),
                    ('br_caster_rotation_link', 'r_forearm_roll_link'),
                    ('br_caster_rotation_link', 'r_gripper_motor_accelerometer_link'),
                    ('br_caster_rotation_link', 'r_shoulder_lift_link'),
                    ('br_caster_rotation_link', 'r_shoulder_pan_link'), ('br_caster_rotation_link', 'r_upper_arm_link'),
                    ('br_caster_rotation_link', 'r_upper_arm_roll_link'),
                    ('br_caster_rotation_link', 'sensor_mount_link'), ('br_caster_rotation_link', 'torso_lift_link'),
                    ('double_stereo_link', 'fl_caster_l_wheel_link'), ('double_stereo_link', 'fl_caster_r_wheel_link'),
                    ('double_stereo_link', 'fl_caster_rotation_link'), ('double_stereo_link', 'fr_caster_l_wheel_link'),
                    ('double_stereo_link', 'fr_caster_r_wheel_link'), ('double_stereo_link', 'fr_caster_rotation_link'),
                    ('double_stereo_link', 'l_elbow_flex_link'), ('double_stereo_link', 'l_forearm_link'),
                    ('double_stereo_link', 'l_forearm_roll_link'), ('double_stereo_link', 'l_gripper_l_finger_link'),
                    ('double_stereo_link', 'l_gripper_l_finger_tip_link'),
                    ('double_stereo_link', 'l_gripper_motor_accelerometer_link'),
                    ('double_stereo_link', 'l_shoulder_lift_link'), ('double_stereo_link', 'l_shoulder_pan_link'),
                    ('double_stereo_link', 'l_upper_arm_link'), ('double_stereo_link', 'l_upper_arm_roll_link'),
                    ('double_stereo_link', 'l_wrist_flex_link'), ('double_stereo_link', 'l_wrist_roll_link'),
                    ('double_stereo_link', 'laser_tilt_mount_link'), ('double_stereo_link', 'r_elbow_flex_link'),
                    ('double_stereo_link', 'r_forearm_link'), ('double_stereo_link', 'r_forearm_roll_link'),
                    ('double_stereo_link', 'r_gripper_l_finger_link'),
                    ('double_stereo_link', 'r_gripper_l_finger_tip_link'),
                    ('double_stereo_link', 'r_gripper_motor_accelerometer_link'),
                    ('double_stereo_link', 'r_gripper_palm_link'), ('double_stereo_link', 'r_gripper_r_finger_link'),
                    ('double_stereo_link', 'r_gripper_r_finger_tip_link'),
                    ('double_stereo_link', 'r_shoulder_lift_link'), ('double_stereo_link', 'r_shoulder_pan_link'),
                    ('double_stereo_link', 'r_upper_arm_link'), ('double_stereo_link', 'r_upper_arm_roll_link'),
                    ('double_stereo_link', 'r_wrist_flex_link'), ('double_stereo_link', 'r_wrist_roll_link'),
                    ('double_stereo_link', 'torso_lift_link'), ('fl_caster_l_wheel_link', 'fl_caster_r_wheel_link'),
                    ('fl_caster_l_wheel_link', 'fr_caster_l_wheel_link'),
                    ('fl_caster_l_wheel_link', 'fr_caster_r_wheel_link'),
                    ('fl_caster_l_wheel_link', 'fr_caster_rotation_link'), ('fl_caster_l_wheel_link', 'head_pan_link'),
                    ('fl_caster_l_wheel_link', 'head_plate_frame'), ('fl_caster_l_wheel_link', 'head_tilt_link'),
                    ('fl_caster_l_wheel_link', 'l_elbow_flex_link'), ('fl_caster_l_wheel_link', 'l_forearm_roll_link'),
                    ('fl_caster_l_wheel_link', 'l_shoulder_lift_link'),
                    ('fl_caster_l_wheel_link', 'l_shoulder_pan_link'), ('fl_caster_l_wheel_link', 'l_upper_arm_link'),
                    ('fl_caster_l_wheel_link', 'l_upper_arm_roll_link'),
                    ('fl_caster_l_wheel_link', 'laser_tilt_mount_link'),
                    ('fl_caster_l_wheel_link', 'r_elbow_flex_link'), ('fl_caster_l_wheel_link', 'r_forearm_link'),
                    ('fl_caster_l_wheel_link', 'r_forearm_roll_link'),
                    ('fl_caster_l_wheel_link', 'r_gripper_motor_accelerometer_link'),
                    ('fl_caster_l_wheel_link', 'r_shoulder_lift_link'),
                    ('fl_caster_l_wheel_link', 'r_shoulder_pan_link'), ('fl_caster_l_wheel_link', 'r_upper_arm_link'),
                    ('fl_caster_l_wheel_link', 'r_upper_arm_roll_link'),
                    ('fl_caster_l_wheel_link', 'r_wrist_flex_link'), ('fl_caster_l_wheel_link', 'r_wrist_roll_link'),
                    ('fl_caster_l_wheel_link', 'sensor_mount_link'), ('fl_caster_l_wheel_link', 'torso_lift_link'),
                    ('fl_caster_r_wheel_link', 'fr_caster_l_wheel_link'),
                    ('fl_caster_r_wheel_link', 'fr_caster_r_wheel_link'),
                    ('fl_caster_r_wheel_link', 'fr_caster_rotation_link'), ('fl_caster_r_wheel_link', 'head_pan_link'),
                    ('fl_caster_r_wheel_link', 'head_plate_frame'), ('fl_caster_r_wheel_link', 'head_tilt_link'),
                    ('fl_caster_r_wheel_link', 'l_elbow_flex_link'), ('fl_caster_r_wheel_link', 'l_forearm_roll_link'),
                    ('fl_caster_r_wheel_link', 'l_shoulder_lift_link'),
                    ('fl_caster_r_wheel_link', 'l_shoulder_pan_link'), ('fl_caster_r_wheel_link', 'l_upper_arm_link'),
                    ('fl_caster_r_wheel_link', 'l_upper_arm_roll_link'),
                    ('fl_caster_r_wheel_link', 'laser_tilt_mount_link'),
                    ('fl_caster_r_wheel_link', 'r_elbow_flex_link'), ('fl_caster_r_wheel_link', 'r_forearm_link'),
                    ('fl_caster_r_wheel_link', 'r_forearm_roll_link'),
                    ('fl_caster_r_wheel_link', 'r_gripper_motor_accelerometer_link'),
                    ('fl_caster_r_wheel_link', 'r_shoulder_lift_link'),
                    ('fl_caster_r_wheel_link', 'r_shoulder_pan_link'), ('fl_caster_r_wheel_link', 'r_upper_arm_link'),
                    ('fl_caster_r_wheel_link', 'r_upper_arm_roll_link'),
                    ('fl_caster_r_wheel_link', 'r_wrist_flex_link'), ('fl_caster_r_wheel_link', 'r_wrist_roll_link'),
                    ('fl_caster_r_wheel_link', 'sensor_mount_link'), ('fl_caster_r_wheel_link', 'torso_lift_link'),
                    ('fl_caster_rotation_link', 'fr_caster_l_wheel_link'),
                    ('fl_caster_rotation_link', 'fr_caster_r_wheel_link'),
                    ('fl_caster_rotation_link', 'fr_caster_rotation_link'),
                    ('fl_caster_rotation_link', 'head_pan_link'), ('fl_caster_rotation_link', 'head_plate_frame'),
                    ('fl_caster_rotation_link', 'head_tilt_link'), ('fl_caster_rotation_link', 'l_elbow_flex_link'),
                    ('fl_caster_rotation_link', 'l_forearm_roll_link'),
                    ('fl_caster_rotation_link', 'l_shoulder_lift_link'),
                    ('fl_caster_rotation_link', 'l_shoulder_pan_link'), ('fl_caster_rotation_link', 'l_upper_arm_link'),
                    ('fl_caster_rotation_link', 'l_upper_arm_roll_link'),
                    ('fl_caster_rotation_link', 'laser_tilt_mount_link'),
                    ('fl_caster_rotation_link', 'r_elbow_flex_link'),
                    ('fl_caster_rotation_link', 'r_forearm_roll_link'),
                    ('fl_caster_rotation_link', 'r_gripper_motor_accelerometer_link'),
                    ('fl_caster_rotation_link', 'r_shoulder_lift_link'),
                    ('fl_caster_rotation_link', 'r_shoulder_pan_link'), ('fl_caster_rotation_link', 'r_upper_arm_link'),
                    ('fl_caster_rotation_link', 'r_upper_arm_roll_link'),
                    ('fl_caster_rotation_link', 'sensor_mount_link'), ('fl_caster_rotation_link', 'torso_lift_link'),
                    ('fr_caster_l_wheel_link', 'fr_caster_r_wheel_link'), ('fr_caster_l_wheel_link', 'head_pan_link'),
                    ('fr_caster_l_wheel_link', 'head_plate_frame'), ('fr_caster_l_wheel_link', 'head_tilt_link'),
                    ('fr_caster_l_wheel_link', 'l_elbow_flex_link'), ('fr_caster_l_wheel_link', 'l_forearm_link'),
                    ('fr_caster_l_wheel_link', 'l_forearm_roll_link'),
                    ('fr_caster_l_wheel_link', 'l_gripper_motor_accelerometer_link'),
                    ('fr_caster_l_wheel_link', 'l_shoulder_lift_link'),
                    ('fr_caster_l_wheel_link', 'l_shoulder_pan_link'), ('fr_caster_l_wheel_link', 'l_upper_arm_link'),
                    ('fr_caster_l_wheel_link', 'l_upper_arm_roll_link'),
                    ('fr_caster_l_wheel_link', 'l_wrist_flex_link'), ('fr_caster_l_wheel_link', 'l_wrist_roll_link'),
                    ('fr_caster_l_wheel_link', 'laser_tilt_mount_link'),
                    ('fr_caster_l_wheel_link', 'r_elbow_flex_link'), ('fr_caster_l_wheel_link', 'r_forearm_roll_link'),
                    ('fr_caster_l_wheel_link', 'r_shoulder_lift_link'),
                    ('fr_caster_l_wheel_link', 'r_shoulder_pan_link'), ('fr_caster_l_wheel_link', 'r_upper_arm_link'),
                    ('fr_caster_l_wheel_link', 'r_upper_arm_roll_link'),
                    ('fr_caster_l_wheel_link', 'sensor_mount_link'), ('fr_caster_l_wheel_link', 'torso_lift_link'),
                    ('fr_caster_r_wheel_link', 'head_pan_link'), ('fr_caster_r_wheel_link', 'head_plate_frame'),
                    ('fr_caster_r_wheel_link', 'head_tilt_link'), ('fr_caster_r_wheel_link', 'l_elbow_flex_link'),
                    ('fr_caster_r_wheel_link', 'l_forearm_link'), ('fr_caster_r_wheel_link', 'l_forearm_roll_link'),
                    ('fr_caster_r_wheel_link', 'l_gripper_motor_accelerometer_link'),
                    ('fr_caster_r_wheel_link', 'l_shoulder_lift_link'),
                    ('fr_caster_r_wheel_link', 'l_shoulder_pan_link'), ('fr_caster_r_wheel_link', 'l_upper_arm_link'),
                    ('fr_caster_r_wheel_link', 'l_upper_arm_roll_link'),
                    ('fr_caster_r_wheel_link', 'l_wrist_flex_link'), ('fr_caster_r_wheel_link', 'l_wrist_roll_link'),
                    ('fr_caster_r_wheel_link', 'laser_tilt_mount_link'),
                    ('fr_caster_r_wheel_link', 'r_elbow_flex_link'), ('fr_caster_r_wheel_link', 'r_forearm_roll_link'),
                    ('fr_caster_r_wheel_link', 'r_shoulder_lift_link'),
                    ('fr_caster_r_wheel_link', 'r_shoulder_pan_link'), ('fr_caster_r_wheel_link', 'r_upper_arm_link'),
                    ('fr_caster_r_wheel_link', 'r_upper_arm_roll_link'),
                    ('fr_caster_r_wheel_link', 'sensor_mount_link'), ('fr_caster_r_wheel_link', 'torso_lift_link'),
                    ('fr_caster_rotation_link', 'head_pan_link'), ('fr_caster_rotation_link', 'head_plate_frame'),
                    ('fr_caster_rotation_link', 'head_tilt_link'), ('fr_caster_rotation_link', 'l_elbow_flex_link'),
                    ('fr_caster_rotation_link', 'l_forearm_roll_link'),
                    ('fr_caster_rotation_link', 'l_gripper_motor_accelerometer_link'),
                    ('fr_caster_rotation_link', 'l_shoulder_lift_link'),
                    ('fr_caster_rotation_link', 'l_shoulder_pan_link'), ('fr_caster_rotation_link', 'l_upper_arm_link'),
                    ('fr_caster_rotation_link', 'l_upper_arm_roll_link'),
                    ('fr_caster_rotation_link', 'laser_tilt_mount_link'),
                    ('fr_caster_rotation_link', 'r_elbow_flex_link'),
                    ('fr_caster_rotation_link', 'r_forearm_roll_link'),
                    ('fr_caster_rotation_link', 'r_shoulder_lift_link'),
                    ('fr_caster_rotation_link', 'r_shoulder_pan_link'), ('fr_caster_rotation_link', 'r_upper_arm_link'),
                    ('fr_caster_rotation_link', 'r_upper_arm_roll_link'),
                    ('fr_caster_rotation_link', 'sensor_mount_link'), ('fr_caster_rotation_link', 'torso_lift_link'),
                    ('head_pan_link', 'l_elbow_flex_link'), ('head_pan_link', 'l_forearm_roll_link'),
                    ('head_pan_link', 'l_gripper_motor_accelerometer_link'), ('head_pan_link', 'l_shoulder_lift_link'),
                    ('head_pan_link', 'l_shoulder_pan_link'), ('head_pan_link', 'l_upper_arm_link'),
                    ('head_pan_link', 'l_upper_arm_roll_link'), ('head_pan_link', 'laser_tilt_mount_link'),
                    ('head_pan_link', 'r_elbow_flex_link'), ('head_pan_link', 'r_forearm_roll_link'),
                    ('head_pan_link', 'r_gripper_motor_accelerometer_link'), ('head_pan_link', 'r_shoulder_lift_link'),
                    ('head_pan_link', 'r_shoulder_pan_link'), ('head_pan_link', 'r_upper_arm_link'),
                    ('head_pan_link', 'r_upper_arm_roll_link'), ('head_pan_link', 'r_wrist_roll_link'),
                    ('head_plate_frame', 'l_elbow_flex_link'), ('head_plate_frame', 'l_forearm_link'),
                    ('head_plate_frame', 'l_forearm_roll_link'), ('head_plate_frame', 'l_gripper_l_finger_link'),
                    ('head_plate_frame', 'l_gripper_l_finger_tip_link'),
                    ('head_plate_frame', 'l_gripper_motor_accelerometer_link'),
                    ('head_plate_frame', 'l_gripper_palm_link'), ('head_plate_frame', 'l_gripper_r_finger_link'),
                    ('head_plate_frame', 'l_gripper_r_finger_tip_link'), ('head_plate_frame', 'l_shoulder_lift_link'),
                    ('head_plate_frame', 'l_shoulder_pan_link'), ('head_plate_frame', 'l_upper_arm_link'),
                    ('head_plate_frame', 'l_upper_arm_roll_link'), ('head_plate_frame', 'l_wrist_flex_link'),
                    ('head_plate_frame', 'l_wrist_roll_link'), ('head_plate_frame', 'laser_tilt_mount_link'),
                    ('head_plate_frame', 'r_elbow_flex_link'), ('head_plate_frame', 'r_forearm_link'),
                    ('head_plate_frame', 'r_forearm_roll_link'), ('head_plate_frame', 'r_gripper_l_finger_link'),
                    ('head_plate_frame', 'r_gripper_l_finger_tip_link'),
                    ('head_plate_frame', 'r_gripper_motor_accelerometer_link'),
                    ('head_plate_frame', 'r_gripper_palm_link'), ('head_plate_frame', 'r_gripper_r_finger_link'),
                    ('head_plate_frame', 'r_gripper_r_finger_tip_link'), ('head_plate_frame', 'r_shoulder_lift_link'),
                    ('head_plate_frame', 'r_shoulder_pan_link'), ('head_plate_frame', 'r_upper_arm_link'),
                    ('head_plate_frame', 'r_upper_arm_roll_link'), ('head_plate_frame', 'r_wrist_flex_link'),
                    ('head_plate_frame', 'r_wrist_roll_link'), ('head_plate_frame', 'torso_lift_link'),
                    ('head_tilt_link', 'l_elbow_flex_link'), ('head_tilt_link', 'l_forearm_roll_link'),
                    ('head_tilt_link', 'l_gripper_motor_accelerometer_link'),
                    ('head_tilt_link', 'l_shoulder_lift_link'), ('head_tilt_link', 'l_shoulder_pan_link'),
                    ('head_tilt_link', 'l_upper_arm_link'), ('head_tilt_link', 'l_upper_arm_roll_link'),
                    ('head_tilt_link', 'laser_tilt_mount_link'), ('head_tilt_link', 'r_elbow_flex_link'),
                    ('head_tilt_link', 'r_forearm_roll_link'), ('head_tilt_link', 'r_gripper_motor_accelerometer_link'),
                    ('head_tilt_link', 'r_shoulder_lift_link'), ('head_tilt_link', 'r_shoulder_pan_link'),
                    ('head_tilt_link', 'r_upper_arm_link'), ('head_tilt_link', 'r_upper_arm_roll_link'),
                    ('head_tilt_link', 'r_wrist_roll_link'), ('head_tilt_link', 'torso_lift_link'),
                    ('l_elbow_flex_link', 'l_forearm_link'), ('l_elbow_flex_link', 'l_gripper_l_finger_link'),
                    ('l_elbow_flex_link', 'l_gripper_l_finger_tip_link'),
                    ('l_elbow_flex_link', 'l_gripper_motor_accelerometer_link'),
                    ('l_elbow_flex_link', 'l_gripper_palm_link'), ('l_elbow_flex_link', 'l_gripper_r_finger_link'),
                    ('l_elbow_flex_link', 'l_gripper_r_finger_tip_link'), ('l_elbow_flex_link', 'l_shoulder_lift_link'),
                    ('l_elbow_flex_link', 'l_shoulder_pan_link'), ('l_elbow_flex_link', 'l_upper_arm_roll_link'),
                    ('l_elbow_flex_link', 'l_wrist_flex_link'), ('l_elbow_flex_link', 'l_wrist_roll_link'),
                    ('l_elbow_flex_link', 'laser_tilt_mount_link'), ('l_elbow_flex_link', 'r_shoulder_lift_link'),
                    ('l_elbow_flex_link', 'r_shoulder_pan_link'), ('l_elbow_flex_link', 'r_upper_arm_roll_link'),
                    ('l_elbow_flex_link', 'sensor_mount_link'), ('l_elbow_flex_link', 'torso_lift_link'),
                    ('l_forearm_link', 'l_gripper_l_finger_link'), ('l_forearm_link', 'l_gripper_l_finger_tip_link'),
                    ('l_forearm_link', 'l_gripper_motor_accelerometer_link'), ('l_forearm_link', 'l_gripper_palm_link'),
                    ('l_forearm_link', 'l_gripper_r_finger_link'), ('l_forearm_link', 'l_gripper_r_finger_tip_link'),
                    ('l_forearm_link', 'l_shoulder_lift_link'), ('l_forearm_link', 'l_upper_arm_link'),
                    ('l_forearm_link', 'l_upper_arm_roll_link'), ('l_forearm_link', 'l_wrist_roll_link'),
                    ('l_forearm_link', 'sensor_mount_link'), ('l_forearm_roll_link', 'l_gripper_l_finger_link'),
                    ('l_forearm_roll_link', 'l_gripper_l_finger_tip_link'),
                    ('l_forearm_roll_link', 'l_gripper_motor_accelerometer_link'),
                    ('l_forearm_roll_link', 'l_gripper_palm_link'), ('l_forearm_roll_link', 'l_gripper_r_finger_link'),
                    ('l_forearm_roll_link', 'l_gripper_r_finger_tip_link'),
                    ('l_forearm_roll_link', 'l_shoulder_lift_link'), ('l_forearm_roll_link', 'l_shoulder_pan_link'),
                    ('l_forearm_roll_link', 'l_upper_arm_link'), ('l_forearm_roll_link', 'l_upper_arm_roll_link'),
                    ('l_forearm_roll_link', 'l_wrist_flex_link'), ('l_forearm_roll_link', 'l_wrist_roll_link'),
                    ('l_forearm_roll_link', 'laser_tilt_mount_link'), ('l_forearm_roll_link', 'r_shoulder_lift_link'),
                    ('l_forearm_roll_link', 'r_shoulder_pan_link'), ('l_forearm_roll_link', 'r_upper_arm_roll_link'),
                    ('l_forearm_roll_link', 'sensor_mount_link'), ('l_forearm_roll_link', 'torso_lift_link'),
                    ('l_gripper_l_finger_link', 'l_gripper_motor_accelerometer_link'),
                    ('l_gripper_l_finger_link', 'l_gripper_r_finger_link'),
                    ('l_gripper_l_finger_link', 'l_gripper_r_finger_tip_link'),
                    ('l_gripper_l_finger_link', 'l_shoulder_lift_link'),
                    ('l_gripper_l_finger_link', 'l_upper_arm_link'),
                    ('l_gripper_l_finger_link', 'l_upper_arm_roll_link'),
                    ('l_gripper_l_finger_link', 'l_wrist_flex_link'), ('l_gripper_l_finger_link', 'l_wrist_roll_link'),
                    ('l_gripper_l_finger_link', 'sensor_mount_link'),
                    ('l_gripper_l_finger_tip_link', 'l_gripper_motor_accelerometer_link'),
                    ('l_gripper_l_finger_tip_link', 'l_gripper_palm_link'),
                    ('l_gripper_l_finger_tip_link', 'l_gripper_r_finger_link'),
                    ('l_gripper_l_finger_tip_link', 'l_shoulder_lift_link'),
                    ('l_gripper_l_finger_tip_link', 'l_upper_arm_link'),
                    ('l_gripper_l_finger_tip_link', 'l_upper_arm_roll_link'),
                    ('l_gripper_l_finger_tip_link', 'l_wrist_flex_link'),
                    ('l_gripper_l_finger_tip_link', 'l_wrist_roll_link'),
                    ('l_gripper_l_finger_tip_link', 'sensor_mount_link'),
                    ('l_gripper_motor_accelerometer_link', 'l_gripper_r_finger_link'),
                    ('l_gripper_motor_accelerometer_link', 'l_gripper_r_finger_tip_link'),
                    ('l_gripper_motor_accelerometer_link', 'l_shoulder_lift_link'),
                    ('l_gripper_motor_accelerometer_link', 'l_upper_arm_link'),
                    ('l_gripper_motor_accelerometer_link', 'l_upper_arm_roll_link'),
                    ('l_gripper_motor_accelerometer_link', 'l_wrist_flex_link'),
                    ('l_gripper_motor_accelerometer_link', 'l_wrist_roll_link'),
                    ('l_gripper_motor_accelerometer_link', 'laser_tilt_mount_link'),
                    ('l_gripper_motor_accelerometer_link', 'r_gripper_l_finger_tip_link'),
                    ('l_gripper_motor_accelerometer_link', 'r_gripper_motor_accelerometer_link'),
                    ('l_gripper_motor_accelerometer_link', 'r_gripper_r_finger_tip_link'),
                    ('l_gripper_motor_accelerometer_link', 'r_wrist_flex_link'),
                    ('l_gripper_motor_accelerometer_link', 'r_wrist_roll_link'),
                    ('l_gripper_motor_accelerometer_link', 'sensor_mount_link'),
                    ('l_gripper_palm_link', 'l_gripper_r_finger_tip_link'),
                    ('l_gripper_palm_link', 'l_shoulder_lift_link'), ('l_gripper_palm_link', 'l_upper_arm_link'),
                    ('l_gripper_palm_link', 'l_upper_arm_roll_link'), ('l_gripper_palm_link', 'l_wrist_flex_link'),
                    ('l_gripper_palm_link', 'sensor_mount_link'), ('l_gripper_r_finger_link', 'l_shoulder_lift_link'),
                    ('l_gripper_r_finger_link', 'l_upper_arm_link'),
                    ('l_gripper_r_finger_link', 'l_upper_arm_roll_link'),
                    ('l_gripper_r_finger_link', 'l_wrist_flex_link'), ('l_gripper_r_finger_link', 'l_wrist_roll_link'),
                    ('l_gripper_r_finger_link', 'sensor_mount_link'),
                    ('l_gripper_r_finger_tip_link', 'l_shoulder_lift_link'),
                    ('l_gripper_r_finger_tip_link', 'l_upper_arm_link'),
                    ('l_gripper_r_finger_tip_link', 'l_upper_arm_roll_link'),
                    ('l_gripper_r_finger_tip_link', 'l_wrist_flex_link'),
                    ('l_gripper_r_finger_tip_link', 'l_wrist_roll_link'),
                    ('l_gripper_r_finger_tip_link', 'r_gripper_motor_accelerometer_link'),
                    ('l_gripper_r_finger_tip_link', 'sensor_mount_link'), ('l_shoulder_lift_link', 'l_upper_arm_link'),
                    ('l_shoulder_lift_link', 'l_wrist_flex_link'), ('l_shoulder_lift_link', 'l_wrist_roll_link'),
                    ('l_shoulder_lift_link', 'laser_tilt_mount_link'), ('l_shoulder_lift_link', 'r_elbow_flex_link'),
                    ('l_shoulder_lift_link', 'r_forearm_roll_link'), ('l_shoulder_lift_link', 'r_shoulder_lift_link'),
                    ('l_shoulder_lift_link', 'r_upper_arm_link'), ('l_shoulder_lift_link', 'r_upper_arm_roll_link'),
                    ('l_shoulder_lift_link', 'sensor_mount_link'), ('l_shoulder_lift_link', 'torso_lift_link'),
                    ('l_shoulder_pan_link', 'l_upper_arm_roll_link'), ('l_shoulder_pan_link', 'laser_tilt_mount_link'),
                    ('l_shoulder_pan_link', 'r_elbow_flex_link'), ('l_shoulder_pan_link', 'r_forearm_roll_link'),
                    ('l_shoulder_pan_link', 'sensor_mount_link'), ('l_upper_arm_link', 'l_wrist_flex_link'),
                    ('l_upper_arm_link', 'l_wrist_roll_link'), ('l_upper_arm_link', 'laser_tilt_mount_link'),
                    ('l_upper_arm_link', 'r_shoulder_lift_link'), ('l_upper_arm_link', 'sensor_mount_link'),
                    ('l_upper_arm_link', 'torso_lift_link'), ('l_upper_arm_roll_link', 'l_wrist_flex_link'),
                    ('l_upper_arm_roll_link', 'l_wrist_roll_link'), ('l_upper_arm_roll_link', 'laser_tilt_mount_link'),
                    ('l_upper_arm_roll_link', 'r_elbow_flex_link'), ('l_upper_arm_roll_link', 'r_forearm_roll_link'),
                    ('l_upper_arm_roll_link', 'r_gripper_motor_accelerometer_link'),
                    ('l_upper_arm_roll_link', 'r_shoulder_lift_link'), ('l_upper_arm_roll_link', 'r_upper_arm_link'),
                    ('l_upper_arm_roll_link', 'r_upper_arm_roll_link'), ('l_upper_arm_roll_link', 'sensor_mount_link'),
                    ('l_upper_arm_roll_link', 'torso_lift_link'), ('l_wrist_flex_link', 'sensor_mount_link'),
                    ('l_wrist_roll_link', 'laser_tilt_mount_link'), ('l_wrist_roll_link', 'sensor_mount_link'),
                    ('laser_tilt_mount_link', 'r_elbow_flex_link'), ('laser_tilt_mount_link', 'r_forearm_roll_link'),
                    ('laser_tilt_mount_link', 'r_gripper_motor_accelerometer_link'),
                    ('laser_tilt_mount_link', 'r_shoulder_lift_link'), ('laser_tilt_mount_link', 'r_shoulder_pan_link'),
                    ('laser_tilt_mount_link', 'r_upper_arm_link'), ('laser_tilt_mount_link', 'r_upper_arm_roll_link'),
                    ('laser_tilt_mount_link', 'r_wrist_roll_link'), ('laser_tilt_mount_link', 'sensor_mount_link'),
                    ('r_elbow_flex_link', 'r_forearm_link'), ('r_elbow_flex_link', 'r_gripper_l_finger_link'),
                    ('r_elbow_flex_link', 'r_gripper_l_finger_tip_link'),
                    ('r_elbow_flex_link', 'r_gripper_motor_accelerometer_link'),
                    ('r_elbow_flex_link', 'r_gripper_palm_link'), ('r_elbow_flex_link', 'r_gripper_r_finger_link'),
                    ('r_elbow_flex_link', 'r_gripper_r_finger_tip_link'), ('r_elbow_flex_link', 'r_shoulder_lift_link'),
                    ('r_elbow_flex_link', 'r_shoulder_pan_link'), ('r_elbow_flex_link', 'r_upper_arm_roll_link'),
                    ('r_elbow_flex_link', 'r_wrist_flex_link'), ('r_elbow_flex_link', 'r_wrist_roll_link'),
                    ('r_elbow_flex_link', 'sensor_mount_link'), ('r_elbow_flex_link', 'torso_lift_link'),
                    ('r_forearm_link', 'r_gripper_l_finger_link'), ('r_forearm_link', 'r_gripper_l_finger_tip_link'),
                    ('r_forearm_link', 'r_gripper_motor_accelerometer_link'), ('r_forearm_link', 'r_gripper_palm_link'),
                    ('r_forearm_link', 'r_gripper_r_finger_link'), ('r_forearm_link', 'r_gripper_r_finger_tip_link'),
                    ('r_forearm_link', 'r_shoulder_lift_link'), ('r_forearm_link', 'r_upper_arm_link'),
                    ('r_forearm_link', 'r_upper_arm_roll_link'), ('r_forearm_link', 'r_wrist_roll_link'),
                    ('r_forearm_link', 'sensor_mount_link'), ('r_forearm_roll_link', 'r_gripper_l_finger_link'),
                    ('r_forearm_roll_link', 'r_gripper_l_finger_tip_link'),
                    ('r_forearm_roll_link', 'r_gripper_motor_accelerometer_link'),
                    ('r_forearm_roll_link', 'r_gripper_palm_link'), ('r_forearm_roll_link', 'r_gripper_r_finger_link'),
                    ('r_forearm_roll_link', 'r_gripper_r_finger_tip_link'),
                    ('r_forearm_roll_link', 'r_shoulder_lift_link'), ('r_forearm_roll_link', 'r_shoulder_pan_link'),
                    ('r_forearm_roll_link', 'r_upper_arm_link'), ('r_forearm_roll_link', 'r_upper_arm_roll_link'),
                    ('r_forearm_roll_link', 'r_wrist_flex_link'), ('r_forearm_roll_link', 'r_wrist_roll_link'),
                    ('r_forearm_roll_link', 'sensor_mount_link'), ('r_forearm_roll_link', 'torso_lift_link'),
                    ('r_gripper_l_finger_link', 'r_gripper_motor_accelerometer_link'),
                    ('r_gripper_l_finger_link', 'r_gripper_r_finger_link'),
                    ('r_gripper_l_finger_link', 'r_gripper_r_finger_tip_link'),
                    ('r_gripper_l_finger_link', 'r_shoulder_lift_link'),
                    ('r_gripper_l_finger_link', 'r_upper_arm_link'),
                    ('r_gripper_l_finger_link', 'r_upper_arm_roll_link'),
                    ('r_gripper_l_finger_link', 'r_wrist_flex_link'), ('r_gripper_l_finger_link', 'r_wrist_roll_link'),
                    ('r_gripper_l_finger_link', 'sensor_mount_link'),
                    ('r_gripper_l_finger_tip_link', 'r_gripper_motor_accelerometer_link'),
                    ('r_gripper_l_finger_tip_link', 'r_gripper_palm_link'),
                    ('r_gripper_l_finger_tip_link', 'r_gripper_r_finger_link'),
                    ('r_gripper_l_finger_tip_link', 'r_shoulder_lift_link'),
                    ('r_gripper_l_finger_tip_link', 'r_upper_arm_link'),
                    ('r_gripper_l_finger_tip_link', 'r_upper_arm_roll_link'),
                    ('r_gripper_l_finger_tip_link', 'r_wrist_flex_link'),
                    ('r_gripper_l_finger_tip_link', 'r_wrist_roll_link'),
                    ('r_gripper_l_finger_tip_link', 'sensor_mount_link'),
                    ('r_gripper_motor_accelerometer_link', 'r_gripper_r_finger_link'),
                    ('r_gripper_motor_accelerometer_link', 'r_gripper_r_finger_tip_link'),
                    ('r_gripper_motor_accelerometer_link', 'r_shoulder_lift_link'),
                    ('r_gripper_motor_accelerometer_link', 'r_upper_arm_link'),
                    ('r_gripper_motor_accelerometer_link', 'r_upper_arm_roll_link'),
                    ('r_gripper_motor_accelerometer_link', 'r_wrist_flex_link'),
                    ('r_gripper_motor_accelerometer_link', 'r_wrist_roll_link'),
                    ('r_gripper_motor_accelerometer_link', 'sensor_mount_link'),
                    ('r_gripper_palm_link', 'r_gripper_r_finger_tip_link'),
                    ('r_gripper_palm_link', 'r_shoulder_lift_link'), ('r_gripper_palm_link', 'r_upper_arm_link'),
                    ('r_gripper_palm_link', 'r_upper_arm_roll_link'), ('r_gripper_palm_link', 'r_wrist_flex_link'),
                    ('r_gripper_palm_link', 'sensor_mount_link'), ('r_gripper_r_finger_link', 'r_shoulder_lift_link'),
                    ('r_gripper_r_finger_link', 'r_upper_arm_link'),
                    ('r_gripper_r_finger_link', 'r_upper_arm_roll_link'),
                    ('r_gripper_r_finger_link', 'r_wrist_flex_link'), ('r_gripper_r_finger_link', 'r_wrist_roll_link'),
                    ('r_gripper_r_finger_link', 'sensor_mount_link'),
                    ('r_gripper_r_finger_tip_link', 'r_shoulder_lift_link'),
                    ('r_gripper_r_finger_tip_link', 'r_upper_arm_link'),
                    ('r_gripper_r_finger_tip_link', 'r_upper_arm_roll_link'),
                    ('r_gripper_r_finger_tip_link', 'r_wrist_flex_link'),
                    ('r_gripper_r_finger_tip_link', 'r_wrist_roll_link'),
                    ('r_gripper_r_finger_tip_link', 'sensor_mount_link'), ('r_shoulder_lift_link', 'r_upper_arm_link'),
                    ('r_shoulder_lift_link', 'r_wrist_flex_link'), ('r_shoulder_lift_link', 'r_wrist_roll_link'),
                    ('r_shoulder_lift_link', 'sensor_mount_link'), ('r_shoulder_lift_link', 'torso_lift_link'),
                    ('r_shoulder_pan_link', 'r_upper_arm_roll_link'), ('r_shoulder_pan_link', 'sensor_mount_link'),
                    ('r_upper_arm_link', 'r_wrist_flex_link'), ('r_upper_arm_link', 'r_wrist_roll_link'),
                    ('r_upper_arm_link', 'sensor_mount_link'), ('r_upper_arm_link', 'torso_lift_link'),
                    ('r_upper_arm_roll_link', 'r_wrist_flex_link'), ('r_upper_arm_roll_link', 'r_wrist_roll_link'),
                    ('r_upper_arm_roll_link', 'sensor_mount_link'), ('r_upper_arm_roll_link', 'torso_lift_link'),
                    ('r_wrist_flex_link', 'sensor_mount_link'), ('r_wrist_roll_link', 'sensor_mount_link'),
                    ('sensor_mount_link', 'torso_lift_link')
] + [
    ('l_upper_arm_link', 'l_shoulder_pan_link'),
    ('r_upper_arm_link', 'r_shoulder_pan_link'),
]
