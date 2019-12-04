#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import numpy as np
import time

from itertools import combinations

from pybullet_tools.pr2_utils import get_top_grasps, get_side_grasps, close_until_collision
from pybullet_tools.ikfast.utils import IKFastInfo
from pybullet_tools.utils import add_data_path, connect, dump_body, load_model, disconnect, wait_for_user, \
    get_movable_joints, get_sample_fn, set_joint_positions, get_joint_name, LockRenderer, link_from_name, HideOutput, \
    joints_from_names, has_joint, set_color, get_links, BLACK, get_max_limits, get_min_limits, apply_alpha, \
    get_extend_fn, get_link_pose, get_joint_names, draw_pose, remove_handles, draw_base_limits, \
    pairwise_link_collision, get_all_links, get_link_name, are_links_adjacent, elapsed_time, create_box, RED, \
    unit_pose, multiply, set_pose
from examples.test_franka import test_retraction
from pybullet_tools.ikfast.franka_panda.ik import ikfast_inverse_kinematics, get_ik_joints

#MOVO_URDF = "models/movo_description/movo.urdf"
#MOVO_URDF = "models/movo_description/movo_lis.urdf"
#MOVO_URDF = "models/movo_description/movo_robotiq.urdf"
MOVO_URDF = "models/movo_description/movo_robotiq_collision.urdf"

# https://github.mit.edu/Learning-and-Intelligent-Systems/ltamp_pr2/blob/master/control_tools/ik/ik_tools/movo_ik/movo_robotiq.urdf
# https://github.com/Learning-and-Intelligent-Systems/movo_ws/blob/master/src/kinova-movo-bare/movo_common/movo_description/urdf/movo.custom.urdf
# https://github.mit.edu/Learning-and-Intelligent-Systems/ltamp_pr2/tree/master/control_tools/ik/ik_tools/movo_ik

#####################################

LEFT = 'left' # KG3
RIGHT = 'right' # ROBOTIQ

ARMS = [RIGHT, LEFT]

BASE_JOINTS = ['x', 'y', 'theta']
TORSO_JOINTS = ['linear_joint']
HEAD_JOINTS = ['pan_joint', 'tilt_joint']

ARM_JOINTS = ['{}_shoulder_pan_joint', '{}_shoulder_lift_joint', '{}_arm_half_joint', '{}_elbow_joint',
              '{}_wrist_spherical_1_joint', '{}_wrist_spherical_2_joint', '{}_wrist_3_joint']

KG3_GRIPPER_JOINTS = ['{}_gripper_finger1_joint', '{}_gripper_finger2_joint', '{}_gripper_finger3_joint']

ROBOTIQ_GRIPPER_JOINTS = ['{}_gripper_finger1_joint', '{}_gripper_finger2_joint',
                          '{}_gripper_finger1_inner_knuckle_joint', '{}_gripper_finger1_finger_tip_joint',
                          '{}_gripper_finger2_inner_knuckle_joint', '{}_gripper_finger2_finger_tip_joint']

EE_LINK = '{}_ee_link'
TOOL_LINK = '{}_tool_link'

#PASSIVE_JOINTS = ['mid_body_joint']
# TODO: mid_body_joint - might be passive
# https://github.com/Kinovarobotics/kinova-movo/blob/master/movo_moveit_config/config/movo_kg2.srdf

# https://github.mit.edu/Learning-and-Intelligent-Systems/ltamp_pr2/blob/master/control_tools/ik/ik_tools/movo_ik/movo_ik_generator.py
MOVO_INFOS = {
    arm: IKFastInfo(module_name='movo.movo_{}_arm_ik'.format(arm), base_link='base_link', ee_link=EE_LINK.format(arm),
                    free_joints=['linear_joint', '{}_arm_half_joint'.format(arm)]) for arm in ARMS}

MOVO_COLOR = apply_alpha(0.25*np.ones(3), 1)

#####################################

def names_from_templates(templates, *args):
    return [template.format(*args) for template in templates]

def get_arm_joints(robot, arm):
    assert arm in ARMS
    return joints_from_names(robot, names_from_templates(ARM_JOINTS, arm))

def has_kg3_gripper(robot, arm):
    assert arm in ARMS
    return all(has_joint(robot, joint_name) for joint_name in names_from_templates(KG3_GRIPPER_JOINTS, arm))

def has_robotiq_gripper(robot, arm):
    assert arm in ARMS
    return all(has_joint(robot, joint_name) for joint_name in names_from_templates(ROBOTIQ_GRIPPER_JOINTS, arm))

def get_gripper_joints(robot, arm):
    assert arm in ARMS
    if has_kg3_gripper(robot, arm):
        return joints_from_names(robot, names_from_templates(KG3_GRIPPER_JOINTS, arm))
    elif has_robotiq_gripper(robot, arm):
        return joints_from_names(robot, names_from_templates(ROBOTIQ_GRIPPER_JOINTS, arm))
    raise ValueError(arm)

def get_open_positions(robot, arm):
    assert arm in ARMS
    joints = get_gripper_joints(robot, arm)
    if has_kg3_gripper(robot, arm):
        return get_min_limits(robot, joints)
    elif has_robotiq_gripper(robot, arm):
        return 6 * [0.]
    raise ValueError(arm)

def get_closed_positions(robot, arm):
    assert arm in ARMS
    joints = get_gripper_joints(robot, arm)
    if has_kg3_gripper(robot, arm):
        return get_max_limits(robot, joints)
    elif has_robotiq_gripper(robot, arm):
        return [0.32]*6
    raise ValueError(arm)

#####################################

def test_close_gripper(robot, arm):
    gripper_joints = get_gripper_joints(robot, arm)
    extend_fn = get_extend_fn(robot, gripper_joints)
    for positions in extend_fn(get_open_positions(robot, arm), get_closed_positions(robot, arm)):
        set_joint_positions(robot, gripper_joints, positions)
        print(positions)
        wait_for_user('Continue?')

#####################################

def get_colliding(robot):
    disabled = []
    for link1, link2 in combinations(get_all_links(robot), r=2):
        if not are_links_adjacent(robot, link1, link2) and pairwise_link_collision(robot, link1, robot, link2):
            disabled.append((get_link_name(robot, link1), get_link_name(robot, link2)))
    return disabled

NEVER_COLLISIONS = [
    ('linear_actuator_fixed_link', 'right_base_link'), ('linear_actuator_fixed_link', 'right_shoulder_link'),
    ('linear_actuator_fixed_link', 'left_base_link'), ('linear_actuator_fixed_link', 'left_shoulder_link'),
    ('linear_actuator_fixed_link', 'front_laser_link'), ('linear_actuator_fixed_link', 'rear_laser_link'),
    ('linear_actuator_link', 'pan_link'), ('linear_actuator_link', 'right_shoulder_link'),
    ('linear_actuator_link', 'right_arm_half_1_link'), ('linear_actuator_link', 'left_shoulder_link'),
    ('linear_actuator_link', 'left_arm_half_1_link'), ('right_wrist_spherical_2_link', 'right_robotiq_coupler_link'),
    ('right_wrist_3_link', 'right_robotiq_coupler_link'), ('right_wrist_3_link', 'right_gripper_base_link'),
    ('right_gripper_finger1_finger_link', 'right_gripper_finger1_finger_tip_link'),
    ('right_gripper_finger2_finger_link', 'right_gripper_finger2_finger_tip_link'),
    ('left_wrist_spherical_2_link', 'left_gripper_base_link'), ('left_wrist_3_link', 'left_gripper_base_link'),
]

#####################################

def main():
    # The URDF loader seems robust to package:// and slightly wrong relative paths?
    connect(use_gui=True)
    add_data_path()
    plane = p.loadURDF("plane.urdf")
    side = 0.05
    block = create_box(w=side, l=side, h=side, color=RED)

    with LockRenderer():
        start_time = time.time()
        with HideOutput():
            # TODO: MOVO must be loaded last
            robot = load_model(MOVO_URDF, fixed_base=True)
        print(elapsed_time(start_time))
        for link in get_links(robot):
            set_color(robot, color=MOVO_COLOR, link=link)
        base_joints = joints_from_names(robot, BASE_JOINTS)
        draw_base_limits((get_min_limits(robot, base_joints),
                          get_max_limits(robot, base_joints)), z=1e-2)

    dump_body(robot)
    #print(get_colliding(robot))
    #for arm in ARMS:
    #    test_close_gripper(robot, arm)

    for arm in ARMS:
        gripper_joints = get_gripper_joints(robot, arm)
        tool_link = link_from_name(robot, TOOL_LINK.format(arm))
        tool_pose = get_link_pose(robot, tool_link)
        #handles = draw_pose(tool_pose)
        #grasps = get_top_grasps(block, under=True, tool_pose=unit_pose())
        grasps = get_side_grasps(block, under=True, tool_pose=unit_pose())
        for i, grasp_pose in enumerate(grasps):
            block_pose = multiply(tool_pose, grasp_pose)
            set_pose(block, block_pose)
            close_until_collision(robot, gripper_joints, bodies=[block], open_conf=get_open_positions(robot, arm),
                                  closed_conf=get_closed_positions(robot, arm))
            handles = draw_pose(block_pose)
            wait_for_user('Grasp {}'.format(i))
            remove_handles(handles)
    wait_for_user('Start?')

    #joint_names = HEAD_JOINTS
    #joints = joints_from_names(robot, joint_names)
    joints = base_joints + get_arm_joints(robot, arm)
    #joints = get_movable_joints(robot)
    print('Joints:', get_joint_names(robot, joints))

    ik_joints = get_ik_joints(robot, MOVO_INFOS[arm], tool_link)
    #fixed_joints = []
    fixed_joints = ik_joints[:1]
    #fixed_joints = ik_joints

    sample_fn = get_sample_fn(robot, joints)
    handles = []
    for i in range(10):
        print('Iteration:', i)
        conf = sample_fn()
        print(conf)
        set_joint_positions(robot, joints, conf)
        tool_pose = get_link_pose(robot, tool_link)
        remove_handles(handles)
        handles = draw_pose(tool_pose)
        wait_for_user()

        #conf = next(ikfast_inverse_kinematics(robot, MOVO_INFOS[arm], tool_link, tool_pose,
        #                                      fixed_joints=fixed_joints, max_time=0.1), None)
        #if conf is not None:
        #    set_joint_positions(robot, ik_joints, conf)
        #wait_for_user()
        test_retraction(robot, MOVO_INFOS[arm], tool_link,
                        fixed_joints=fixed_joints, max_time=0.1)
    disconnect()

if __name__ == '__main__':
    main()
