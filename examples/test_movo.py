#!/usr/bin/env python

from __future__ import print_function

import pybullet as p

from pybullet_tools.utils import add_data_path, connect, dump_body, load_model, disconnect, wait_for_user, \
    get_movable_joints, get_sample_fn, set_joint_positions, get_joint_name, joints_from_names, get_configuration, LockRenderer

MOVO_URDF = "models/movo_description/movo.urdf"

HEAD_JOINTS = ['pan_joint', 'tilt_joint']
LEFT_ARM_JOINTS = ['left_shoulder_pan_joint', 'left_shoulder_lift_joint', 'left_arm_half_joint', 'left_elbow_joint',
                   'left_wrist_spherical_1_joint', 'left_wrist_spherical_2_joint', 'left_wrist_3_joint']
LEFT_GRIPPER_JOINTS = ['left_gripper_finger1_joint', 'left_gripper_finger2_joint', 'left_gripper_finger3_joint']
RIGHT_ARM_JOINTS = ['right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_arm_half_joint', 'right_elbow_joint',
                    'right_wrist_spherical_1_joint', 'right_wrist_spherical_2_joint', 'right_wrist_3_joint']
RIGHT_GRIPPER_JOINTS = ['right_gripper_finger1_joint', 'right_gripper_finger2_joint', 'right_gripper_finger3_joint']
TORSO_JOINTS = ['linear_joint']

# TODO: mid_body_joint - might be passive
# https://github.com/Kinovarobotics/kinova-movo/blob/master/movo_moveit_config/config/movo_kg2.srdf
def main():
    # The URDF loader seems robust to package:// and slightly wrong relative paths?
    connect(use_gui=True)
    add_data_path()

    plane = p.loadURDF("plane.urdf")
    #with HideOutput():
    with LockRenderer():
        robot = load_model(MOVO_URDF, fixed_base=True)
    dump_body(robot)
    print('Start?')
    wait_for_user()

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
