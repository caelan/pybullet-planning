#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import time
import numpy as np

from pybullet_tools.pr2_utils import TOP_HOLDING_LEFT_ARM, PR2_URDF, DRAKE_PR2_URDF, \
    SIDE_HOLDING_LEFT_ARM, PR2_GROUPS, open_arm, get_disabled_collisions, REST_LEFT_ARM, rightarm_from_leftarm
from pybullet_tools.utils import set_base_values, joint_from_name, quat_from_euler, set_joint_position, \
    set_joint_positions, add_data_path, connect, plan_base_motion, plan_joint_motion, enable_gravity, \
    joint_controller, dump_body, load_model, joints_from_names, wait_if_gui, disconnect, get_joint_positions, \
    get_link_pose, link_from_name, HideOutput, get_pose, wait_if_gui, load_pybullet, set_quat, Euler, PI, RED, add_line, \
    wait_for_duration, LockRenderer, base_aligned_z, Point, set_point, get_aabb, stable_z_on_aabb, AABB

# TODO: consider making this a function
SLEEP = None # None | 0.05


def test_base_motion(pr2, base_start, base_goal, obstacles=[]):
    #disabled_collisions = get_disabled_collisions(pr2)
    set_base_values(pr2, base_start)
    wait_if_gui('Plan Base?')
    base_limits = ((-2.5, -2.5), (2.5, 2.5))
    with LockRenderer(lock=False):
        base_path = plan_base_motion(pr2, base_goal, base_limits, obstacles=obstacles)
    if base_path is None:
        print('Unable to find a base path')
        return
    print(len(base_path))
    for bq in base_path:
        set_base_values(pr2, bq)
        if SLEEP is None:
            wait_if_gui('Continue?')
        else:
            wait_for_duration(SLEEP)

def test_drake_base_motion(pr2, base_start, base_goal, obstacles=[]):
    # TODO: combine this with test_arm_motion
    """
    Drake's PR2 URDF has explicit base joints
    """
    disabled_collisions = get_disabled_collisions(pr2)
    base_joints = [joint_from_name(pr2, name) for name in PR2_GROUPS['base']]
    set_joint_positions(pr2, base_joints, base_start)
    base_joints = base_joints[:2]
    base_goal = base_goal[:len(base_joints)]
    wait_if_gui('Plan Base?')
    with LockRenderer(lock=False):
        base_path = plan_joint_motion(pr2, base_joints, base_goal, obstacles=obstacles,
                                      disabled_collisions=disabled_collisions)
    if base_path is None:
        print('Unable to find a base path')
        return
    print(len(base_path))
    for bq in base_path:
        set_joint_positions(pr2, base_joints, bq)
        if SLEEP is None:
            wait_if_gui('Continue?')
        else:
            wait_for_duration(SLEEP)

#####################################

def test_arm_motion(pr2, left_joints, arm_goal):
    disabled_collisions = get_disabled_collisions(pr2)
    wait_if_gui('Plan Arm?')
    with LockRenderer(lock=False):
        arm_path = plan_joint_motion(pr2, left_joints, arm_goal, disabled_collisions=disabled_collisions)
    if arm_path is None:
        print('Unable to find an arm path')
        return
    print(len(arm_path))
    for q in arm_path:
        set_joint_positions(pr2, left_joints, q)
        #wait_if_gui('Continue?')
        wait_for_duration(0.01)

def test_arm_control(pr2, left_joints, arm_start):
    wait_if_gui('Control Arm?')
    real_time = False
    enable_gravity()
    p.setRealTimeSimulation(real_time)
    for _ in joint_controller(pr2, left_joints, arm_start):
        if not real_time:
            p.stepSimulation()
        #wait_for_duration(0.01)

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
            wait_if_gui()

#####################################

def main(use_pr2_drake=True):
    connect(use_gui=True)
    add_data_path()

    plane = p.loadURDF("plane.urdf")
    table_path = "models/table_collision/table.urdf"
    table = load_pybullet(table_path, fixed_base=True)
    set_quat(table, quat_from_euler(Euler(yaw=PI/2)))
    # table/table.urdf, table_square/table_square.urdf, cube.urdf, block.urdf, door.urdf
    obstacles = [plane, table]

    pr2_urdf = DRAKE_PR2_URDF if use_pr2_drake else PR2_URDF
    with HideOutput():
        pr2 = load_model(pr2_urdf, fixed_base=True) # TODO: suppress warnings?
    dump_body(pr2)

    z = base_aligned_z(pr2)
    print(z)
    #z = stable_z_on_aabb(pr2, AABB(np.zeros(3), np.zeros(3)))
    print(z)

    set_point(pr2, Point(z=z))
    print(get_aabb(pr2))
    wait_if_gui()

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

    add_line(base_start, base_goal, color=RED)
    print(base_start, base_goal)
    if use_pr2_drake:
        test_drake_base_motion(pr2, base_start, base_goal, obstacles=obstacles)
    else:
        test_base_motion(pr2, base_start, base_goal, obstacles=obstacles)

    test_arm_motion(pr2, left_joints, arm_goal)
    # test_arm_control(pr2, left_joints, arm_start)

    wait_if_gui('Finish?')
    disconnect()

if __name__ == '__main__':
    main()