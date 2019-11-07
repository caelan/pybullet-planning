#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import numpy as np

from pybullet_tools.ikfast.utils import IKFastInfo
from pybullet_tools.utils import add_data_path, connect, dump_body, load_model, disconnect, wait_for_user, \
    get_movable_joints, get_sample_fn, set_joint_positions, get_joint_name, LockRenderer, link_from_name, HideOutput, \
    joints_from_names, has_joint, set_color, get_links, BLACK, get_max_limits, get_min_limits, apply_alpha, \
    get_extend_fn, get_link_pose, get_joint_names, draw_pose, remove_handles
from examples.test_franka import test_retraction
from pybullet_tools.ikfast.franka_panda.ik import ikfast_inverse_kinematics, get_ik_joints

#MOVO_URDF = "models/movo_description/movo.urdf"
#MOVO_URDF = "models/movo_description/movo_lis.urdf"
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

EE_LINK = '{}_ee_link'

#PASSIVE_JOINTS = ['mid_body_joint']
# TODO: mid_body_joint - might be passive
# https://github.com/Kinovarobotics/kinova-movo/blob/master/movo_moveit_config/config/movo_kg2.srdf

JOINTS = TORSO_JOINTS + HEAD_JOINTS

# https://github.mit.edu/Learning-and-Intelligent-Systems/ltamp_pr2/blob/master/control_tools/ik/ik_tools/movo_ik/movo_ik_generator.py
MOVO_INFOS = {
    arm: IKFastInfo(module_name='movo.movo_{}_arm_ik'.format(arm), base_link='base_link', ee_link=EE_LINK.format(arm),
                    free_joints=['linear_joint', '{}_arm_half_joint'.format(arm)]) for arm in ARMS}

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
    wait_for_user('Start?')

    #for arm in ARMS:
    #    test_close_gripper(robot, arm)

    arm = 'left'
    tool_link = link_from_name(robot, EE_LINK.format(arm))
    #joint_names = HEAD_JOINTS
    #joints = joints_from_names(robot, joint_names)
    #joints = get_arm_joints(robot, arm)
    joints = get_ik_joints(robot, MOVO_INFOS[arm], tool_link)
    #joints = get_movable_joints(robot)
    print('Joints:', get_joint_names(robot, joints))

    sample_fn = get_sample_fn(robot, joints)
    handles = []
    for i in range(100):
        print('Iteration:', i)
        conf = sample_fn()
        print(conf)
        set_joint_positions(robot, joints, conf)
        tool_pose = get_link_pose(robot, tool_link)
        remove_handles(handles)
        handles = draw_pose(tool_pose)
        wait_for_user()

        conf = next(ikfast_inverse_kinematics(robot, MOVO_INFOS[arm], tool_link, tool_pose,
                                              max_time=0.1, sample_free=True), None)
        print(conf)
        if conf is not None:
            set_joint_positions(robot, joints, conf)
        wait_for_user()
        #test_retraction(robot, MOVO_INFOS[arm], joints, tool_link)
    disconnect()

if __name__ == '__main__':
    main()
