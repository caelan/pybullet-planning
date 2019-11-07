#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import numpy as np

from pybullet_tools.utils import add_data_path, connect, dump_body, load_model, disconnect, wait_for_user, \
    get_movable_joints, get_sample_fn, set_joint_positions, get_joint_name, LockRenderer, HideOutput, \
    joints_from_names, has_joint, set_color, get_links, BLACK, get_max_limits, get_min_limits, apply_alpha, get_extend_fn

#MOVO_URDF = "models/movo_description/movo.urdf"
MOVO_URDF = "models/movo_description/movo_lis.urdf"
MOVO_URDF = "models/movo_description/movo_robotiq.urdf"

# https://github.mit.edu/Learning-and-Intelligent-Systems/ltamp_pr2/blob/master/control_tools/ik/ik_tools/movo_ik/movo_robotiq.urdf
# https://github.com/Learning-and-Intelligent-Systems/movo_ws/blob/master/src/kinova-movo-bare/movo_common/movo_description/urdf/movo.custom.urdf
# https://github.mit.edu/Learning-and-Intelligent-Systems/ltamp_pr2/tree/master/control_tools/ik/ik_tools/movo_ik

#####################################

ARMS = ['right', 'left']

HEAD_JOINTS = ['pan_joint', 'tilt_joint']

ARM_JOINTS = ['{}_shoulder_pan_joint', '{}_shoulder_lift_joint', '{}_arm_half_joint', '{}_elbow_joint',
              '{}_wrist_spherical_1_joint', '{}_wrist_spherical_2_joint', '{}_wrist_3_joint']

KG3_GRIPPER_JOINTS = ['{}_gripper_finger1_joint', '{}_gripper_finger2_joint', '{}_gripper_finger3_joint']

ROBOTIQ_GRIPPER_JOINTS = ['{}_gripper_finger1_joint', '{}_gripper_finger2_joint',
                          '{}_gripper_finger1_inner_knuckle_joint', '{}_gripper_finger1_finger_tip_joint',
                          '{}_gripper_finger2_inner_knuckle_joint', '{}_gripper_finger2_finger_tip_joint']

TORSO_JOINTS = ['linear_joint']

#PASSIVE_JOINTS = ['mid_body_joint']
# TODO: mid_body_joint - might be passive
# https://github.com/Kinovarobotics/kinova-movo/blob/master/movo_moveit_config/config/movo_kg2.srdf

JOINTS = TORSO_JOINTS + HEAD_JOINTS

#####################################

def get_arm_joints(robot, arm):
    assert arm in ARMS
    return joints_from_names(robot, [joint_template.format(arm) for joint_template in ARM_JOINTS])

def has_kg3_gripper(robot, arm):
    assert arm in ARMS
    kg3_joint_names = [joint_template.format(arm) for joint_template in KG3_GRIPPER_JOINTS]
    return all(has_joint(robot, joint_name) for joint_name in kg3_joint_names)

def has_robotiq_gripper(robot, arm):
    assert arm in ARMS
    robotiq_joint_names = [joint_template.format(arm) for joint_template in ROBOTIQ_GRIPPER_JOINTS]
    return all(has_joint(robot, joint_name) for joint_name in robotiq_joint_names)

def get_gripper_joints(robot, arm):
    assert arm in ARMS
    if has_kg3_gripper(robot, arm):
        return joints_from_names(robot, [joint_template.format(arm) for joint_template in KG3_GRIPPER_JOINTS])
    elif has_robotiq_gripper(robot, arm):
        return joints_from_names(robot, [joint_template.format(arm) for joint_template in ROBOTIQ_GRIPPER_JOINTS])
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

def main():
    # The URDF loader seems robust to package:// and slightly wrong relative paths?
    connect(use_gui=True)
    add_data_path()

    plane = p.loadURDF("plane.urdf")
    with HideOutput():
        with LockRenderer():
            robot = load_model(MOVO_URDF, fixed_base=True)
            for link in get_links(robot):
                set_color(robot, color=apply_alpha(0.2*np.ones(3), 1), link=link)

    dump_body(robot)
    print('Start?')
    wait_for_user()

    for arm in ARMS:
        gripper_joints = get_gripper_joints(robot, arm)
        extend_fn = get_extend_fn(robot, gripper_joints)
        for positions in extend_fn(get_open_positions(robot, arm), get_closed_positions(robot, arm)):
            set_joint_positions(robot, gripper_joints, positions)
            print(positions)
            wait_for_user('Continue?')

    #joint_names = HEAD_JOINTS
    #joints = joints_from_names(robot, joint_names)
    joints = get_movable_joints(robot)
    print('Joints', [get_joint_name(robot, joint) for joint in joints])
    sample_fn = get_sample_fn(robot, joints)
    for i in range(10):
        print('Iteration:', i)
        conf = sample_fn()
        set_joint_positions(robot, joints, conf)
        wait_for_user()
    disconnect()

if __name__ == '__main__':
    main()
