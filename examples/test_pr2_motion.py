#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import time

from pybullet_tools.pr2_utils import TOP_HOLDING_LEFT_ARM, PR2_URDF, DRAKE_PR2_URDF, \
    SIDE_HOLDING_LEFT_ARM, PR2_GROUPS, open_arm, get_disabled_collisions, REST_LEFT_ARM, rightarm_from_leftarm
from pybullet_tools.utils import set_base_values, joint_from_name, set_joint_position, \
    set_joint_positions, add_data_path, connect, plan_base_motion, plan_joint_motion, enable_gravity, \
    joint_controller, dump_body, load_model, joints_from_names, user_input, disconnect, get_joint_positions, \
    get_link_pose, link_from_name, HideOutput, get_pose, wait_for_user


def test_base_motion(pr2, base_start, base_goal):
    #disabled_collisions = get_disabled_collisions(pr2)
    set_base_values(pr2, base_start)
    user_input('Plan Base?')
    base_limits = ((-2.5, -2.5), (2.5, 2.5))
    base_path = plan_base_motion(pr2, base_goal, base_limits)
    if base_path is None:
        print('Unable to find a base path')
        return
    print(len(base_path))
    for bq in base_path:
        set_base_values(pr2, bq)
        # user_input('Continue?')
        time.sleep(0.05)

def test_drake_base_motion(pr2, base_start, base_goal):
    # TODO: combine this with test_arm_motion
    """
    Drake's PR2 URDF has explicit base joints
    """
    disabled_collisions = get_disabled_collisions(pr2)
    base_joints = [joint_from_name(pr2, name) for name in PR2_GROUPS['base']]
    set_joint_positions(pr2, base_joints, base_start)
    base_joints = base_joints[:2]
    base_goal = base_goal[:len(base_joints)]
    user_input('Plan Base?')
    base_path = plan_joint_motion(pr2, base_joints, base_goal, disabled_collisions=disabled_collisions)
    if base_path is None:
        print('Unable to find a base path')
        return
    print(len(base_path))
    for bq in base_path:
        set_joint_positions(pr2, base_joints, bq)
        user_input('Continue?')
        # time.sleep(0.05)

#####################################

def test_arm_motion(pr2, left_joints, arm_goal):
    disabled_collisions = get_disabled_collisions(pr2)
    user_input('Plan Arm?')
    arm_path = plan_joint_motion(pr2, left_joints, arm_goal, disabled_collisions=disabled_collisions)
    if arm_path is None:
        print('Unable to find an arm path')
        return
    print(len(arm_path))
    for q in arm_path:
        set_joint_positions(pr2, left_joints, q)
        #raw_input('Continue?')
        time.sleep(0.01)

def test_arm_control(pr2, left_joints, arm_start):
    user_input('Control Arm?')
    real_time = False
    enable_gravity()
    p.setRealTimeSimulation(real_time)
    for _ in joint_controller(pr2, left_joints, arm_start):
        if not real_time:
            p.stepSimulation()
        #time.sleep(0.01)

#####################################

def test_ikfast(pr2):
    from pybullet_tools.ikfast.pr2.ik import get_tool_pose, get_ik_generator
    left_joints = joints_from_names(pr2, PR2_GROUPS['left_arm'])
    #right_joints = joints_from_names(pr2, PR2_GROUPS['right_arm'])
    torso_joints = joints_from_names(pr2, PR2_GROUPS['torso'])
    torso_left = torso_joints + left_joints
    print(get_link_pose(pr2, link_from_name(pr2, 'l_gripper_tool_frame')))
    # print(forward_kinematics('left', get_joint_positions(pr2, torso_left)))
    print(get_tool_pose(pr2, 'left'))

    arm = 'left'
    pose = get_tool_pose(pr2, arm)
    generator = get_ik_generator(pr2, arm, pose, torso_limits=False)
    for i in range(100):
        solutions = next(generator)
        print(i, len(solutions))
        for q in solutions:
            set_joint_positions(pr2, torso_left, q)
            wait_for_user()

#####################################

def main(use_pr2_drake=False):
    connect(use_gui=True)
    add_data_path()

    plane = p.loadURDF("plane.urdf")
    #table_path = "table/table.urdf"
    # table_path = "models/table_collision/table.urdf"
    # table = p.loadURDF(table_path, 0, 0, 0, 0, 0, 0.707107, 0.707107)
    # table_square/table_square.urdf, cube.urdf, block.urdf, door.urdf

    pr2_urdf = DRAKE_PR2_URDF if use_pr2_drake else PR2_URDF
    with HideOutput():
        pr2 = load_model(pr2_urdf, fixed_base=True) # TODO: suppress warnings?
    dump_body(pr2)

    base_start = (-2, -2, 0)
    base_goal = (2, 2, 0)
    arm_start = SIDE_HOLDING_LEFT_ARM
    #arm_start = TOP_HOLDING_LEFT_ARM
    #arm_start = REST_LEFT_ARM
    arm_goal = TOP_HOLDING_LEFT_ARM
    #arm_goal = SIDE_HOLDING_LEFT_ARM

    left_joints = joints_from_names(pr2, PR2_GROUPS['left_arm'])
    right_joints = joints_from_names(pr2, PR2_GROUPS['right_arm'])
    torso_joints = joints_from_names(pr2, PR2_GROUPS['torso'])
    set_joint_positions(pr2, left_joints, arm_start)
    set_joint_positions(pr2, right_joints, rightarm_from_leftarm(REST_LEFT_ARM))
    set_joint_positions(pr2, torso_joints, [0.2])
    open_arm(pr2, 'left')
    # test_ikfast(pr2)

    p.addUserDebugLine(base_start, base_goal, lineColorRGB=(1, 0, 0)) # addUserDebugText
    print(base_start, base_goal)
    if use_pr2_drake:
        test_drake_base_motion(pr2, base_start, base_goal)
    else:
        test_base_motion(pr2, base_start, base_goal)

    test_arm_motion(pr2, left_joints, arm_goal)
    # test_arm_control(pr2, left_joints, arm_start)

    user_input('Finish?')
    disconnect()

if __name__ == '__main__':
    main()