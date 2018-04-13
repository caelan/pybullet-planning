#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import time

from pr2_utils import TOP_HOLDING_LEFT_ARM, ARM_JOINT_NAMES, TORSO_JOINT_NAME, \
    SIDE_HOLDING_LEFT_ARM, PR2_GROUPS, open_arm, get_disabled_collisions, \
    load_srdf_collisions, load_dae_collisions, REST_LEFT_ARM, rightarm_from_leftarm
from utils import set_base_values, joint_from_name, set_joint_position, \
    set_joint_positions, add_data_path, connect, plan_base_motion, plan_joint_motion, enable_gravity, input, \
    joint_controller, dump_world, get_link_name, wait_for_interrupt, clone_body, clone_body_editor

def test_base_motion(pr2, base_start, base_goal):
    #disabled_collisions = get_disabled_collisions(pr2)
    set_base_values(pr2, base_start)
    input('Plan Base?')
    base_path = plan_base_motion(pr2, base_goal)
    print(base_path)
    if base_path is None:
        print('Unable to find a base path')
        return
    print(len(base_path))
    for bq in base_path:
        set_base_values(pr2, bq)
        input('Continue?')
        # time.sleep(0.05)

def test_drake_base_motion(pr2, base_start, base_goal):
    # TODO: combine this with test_arm_motion
    """
    Drake's PR2 URDF has explicit base joints
    """
    disabled_collisions = get_disabled_collisions(pr2)
    base_joints = [joint_from_name(pr2, name) for name in PR2_GROUPS['base']]
    set_joint_positions(pr2, base_joints, base_start)
    input('Plan Base?')
    base_path = plan_joint_motion(pr2, base_joints, base_goal, disabled_collisions=disabled_collisions)
    if base_path is None:
        print('Unable to find a base path')
        return
    print(len(base_path))
    for bq in base_path:
        set_joint_positions(pr2, base_joints, bq)
        input('Continue?')
        # time.sleep(0.05)

def test_arm_motion(pr2, left_joints, arm_goal):
    disabled_collisions = get_disabled_collisions(pr2)
    input('Plan Arm?')
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
    input('Control Arm?')
    real_time = False
    enable_gravity()
    p.setRealTimeSimulation(real_time)
    for _ in joint_controller(pr2, left_joints, arm_start):
        if not real_time:
            p.stepSimulation()
        #time.sleep(0.01)

def main(use_pr2_drake=True):
    connect(use_gui=True)
    add_data_path()

    plane = p.loadURDF("plane.urdf")
    #table = p.loadURDF("table/table.urdf", 0, 0, 0, 0, 0, 0.707107, 0.707107)
    #table = p.loadURDF("table_square/table_square.urdf")
    #table = p.loadURDF("cube.urdf")
    #table = p.loadURDF("block.urdf")
    #table = p.loadURDF("door.urdf")
    if use_pr2_drake:
        pr2 = p.loadURDF("models/drake/pr2_description/urdf/pr2_simplified.urdf", useFixedBase=True)
    else:
        pr2 = p.loadURDF("models/pr2_description/pr2.urdf", useFixedBase=False)

    #new_pr2 = clone_body(pr2, visual=True, collision=True)
    #new_pr2 = clone_body_editor(pr2, shapes=False)
    #set_base_values(new_pr2, (2, 0, 0))
    dump_world()
    #print(load_srdf_collisions())
    #print(load_dae_collisions())

    base_start = (-2, -2, 0)
    base_goal = (2, 2, 0)
    arm_start = SIDE_HOLDING_LEFT_ARM
    #arm_start = TOP_HOLDING_LEFT_ARM
    #arm_start = REST_LEFT_ARM
    arm_goal = TOP_HOLDING_LEFT_ARM
    #arm_goal = SIDE_HOLDING_LEFT_ARM

    left_joints = [joint_from_name(pr2, name) for name in ARM_JOINT_NAMES['left']]
    right_joints = [joint_from_name(pr2, name) for name in ARM_JOINT_NAMES['right']]
    set_joint_positions(pr2, left_joints, arm_start)
    set_joint_positions(pr2, right_joints, rightarm_from_leftarm(REST_LEFT_ARM))
    set_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT_NAME), 0.2)
    open_arm(pr2, 'left')
    wait_for_interrupt()

    p.addUserDebugLine(base_start, base_goal, lineColorRGB=(1, 1, 0)) # addUserDebugText
    print(base_start, base_goal)
    #if use_pr2_drake:
    #    test_drake_base_motion(pr2, base_start, base_goal)
    #else:
    #    test_base_motion(pr2, base_start, base_goal)

    test_arm_motion(pr2, left_joints, arm_goal)
    #test_arm_control(pr2, left_joints, arm_start)

    input('Finish?')
    p.disconnect()

if __name__ == '__main__':
    main()