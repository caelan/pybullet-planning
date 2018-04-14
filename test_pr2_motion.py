#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import time

from pr2_utils import TOP_HOLDING_LEFT_ARM, ARM_JOINT_NAMES, TORSO_JOINT_NAME, \
    SIDE_HOLDING_LEFT_ARM, PR2_GROUPS, open_arm, get_disabled_collisions, \
    load_srdf_collisions, load_dae_collisions, REST_LEFT_ARM, rightarm_from_leftarm
from utils import set_base_values, joint_from_name, set_joint_position, \
    set_joint_positions, add_data_path, connect, plan_base_motion, plan_joint_motion, enable_gravity, input, \
    joint_controller, dump_world, get_link_name, wait_for_interrupt, clone_body, clone_body_editor, \
    get_links, get_joint_parent_frame, euler_from_quat, get_joint_inertial_pose, get_joint_info, \
    get_link_pose, VisualShapeData, get_visual_data

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

def debug(pr2):
    # TODO: j toggles frames, p prints timings, w is wire, a is boxes
    new_pr2 = clone_body(pr2, visual=True, collision=False)
    #new_pr2 = clone_body_editor(pr2, visual=True, collision=True)
    dump_world()
    #print(load_srdf_collisions())
    #print(load_dae_collisions())

    # TODO: some unimportant quats are off for both URDF and other

    # TODO: maybe all the frames are actually correct when I load things this way?
    # TODO: the visual geometries are off but not the collision frames?
    import numpy as np
    for link in get_links(pr2):
        continue
        if not get_visual_data(new_pr2, link): # pybullet.error: Error receiving visual shape info?
            continue
        #print(get_link_name(pr2, link))
        data1 = VisualShapeData(*get_visual_data(pr2, link)[0])
        data2 = VisualShapeData(*get_visual_data(new_pr2, link)[0])
        pose1 = (data1.localVisualFrame_position, data1.localVisualFrame_orientation)
        pose2 = (data2.localVisualFrame_position, data2.localVisualFrame_orientation)

        #pose1 = get_link_pose(pr2, link) # Links are fine
        #pose2 = get_link_pose(new_pr2, link)
        #pose1 = get_joint_parent_frame(pr2, link)
        #pose2 = get_joint_parent_frame(new_pr2, link)
        #pose1 = get_joint_inertial_pose(pr2, link) # Inertia is fine
        #pose2 = get_joint_inertial_pose(new_pr2, link)
        if not np.allclose(pose1[0], pose2[0], rtol=0, atol=1e-3):
            print('Point', get_link_name(pr2, link), link, pose1[0], pose2[0])
            #print(data1)
            #print(data2)
            #print(get_joint_parent_frame(pr2, link), get_joint_parent_frame(new_pr2, link))
            print(get_joint_inertial_pose(pr2, link)) #, get_joint_inertial_pose(new_pr2, link))
            print()
        if not np.allclose(euler_from_quat(pose1[1]), euler_from_quat(pose2[1]), rtol=0, atol=1e-3):
            print('Quat', get_link_name(pr2, link), link, euler_from_quat(pose1[1]), euler_from_quat(pose2[1]))
        joint_info1 = get_joint_info(pr2, link)
        joint_info2 = get_joint_info(new_pr2, link)
        # TODO: the axis is off for some of these
        if not np.allclose(joint_info1.jointAxis, joint_info2.jointAxis, rtol=0, atol=1e-3):
            print('Axis', get_link_name(pr2, link), link, joint_info1.jointAxis, joint_info2.jointAxis)
    set_base_values(new_pr2, (2, 0, 0))
    wait_for_interrupt()
    # TODO: the drake one has a large out-of-place cylinder as well

def main(use_pr2_drake=False):
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
    #dump_world()

    #debug(pr2)
    #return


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