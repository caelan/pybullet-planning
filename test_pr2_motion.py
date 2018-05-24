#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import time

from pr2_utils import TOP_HOLDING_LEFT_ARM, TORSO_JOINT_NAME, HEAD_LINK, \
    SIDE_HOLDING_LEFT_ARM, PR2_GROUPS, open_arm, get_disabled_collisions, get_cone_mesh, \
    load_srdf_collisions, load_dae_collisions, REST_LEFT_ARM, rightarm_from_leftarm, inverse_visibility, get_detection_cone
from utils import set_base_values, joint_from_name, set_joint_position, \
    set_joint_positions, add_data_path, connect, plan_base_motion, plan_joint_motion, enable_gravity, input, \
    joint_controller, dump_world, get_link_name, wait_for_interrupt, clone_body, clone_body_editor, \
    get_links, get_joint_parent_frame, euler_from_quat, get_joint_inertial_pose, get_joint_info, \
    get_link_pose, VisualShapeData, get_visual_data, get_link_parent, link_from_name, set_point, set_pose, \
    get_link_ancestors, get_link_children, get_link_descendants, dump_body, Verbose, load_model, create_mesh, \
    sub_inverse_kinematics, point_from_pose, get_pose

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

def main(use_pr2_drake=False):
    connect(use_gui=True)
    add_data_path()

    plane = p.loadURDF("plane.urdf")
    #table_path = "table/table.urdf"
    table_path = "models/table_collision/table.urdf"
    table = p.loadURDF(table_path, 0, 0, 0, 0, 0, 0.707107, 0.707107)
    # table_square/table_square.urdf, cube.urdf, block.urdf, door.urdf

    if use_pr2_drake:
        pr2_urdf = "models/drake/pr2_description/urdf/pr2_simplified.urdf"
    else:
        pr2_urdf = "models/pr2_description/pr2.urdf"
    pr2 = load_model(pr2_urdf, fixed_base=True) # TODO: suppress warnings?
    dump_body(pr2)

    base_start = (-2, -2, 0)
    base_goal = (2, 2, 0)
    arm_start = SIDE_HOLDING_LEFT_ARM
    #arm_start = TOP_HOLDING_LEFT_ARM
    #arm_start = REST_LEFT_ARM
    arm_goal = TOP_HOLDING_LEFT_ARM
    #arm_goal = SIDE_HOLDING_LEFT_ARM

    left_joints = [joint_from_name(pr2, name) for name in PR2_GROUPS['left_arm']]
    right_joints = [joint_from_name(pr2, name) for name in PR2_GROUPS['right_arm']]
    set_joint_positions(pr2, left_joints, arm_start)
    set_joint_positions(pr2, right_joints, rightarm_from_leftarm(REST_LEFT_ARM))
    set_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT_NAME), 0.2)
    open_arm(pr2, 'left')

    p.addUserDebugLine(base_start, base_goal, lineColorRGB=(1, 0, 0)) # addUserDebugText
    print(base_start, base_goal)
    if use_pr2_drake:
        test_drake_base_motion(pr2, base_start, base_goal)
    else:
        test_base_motion(pr2, base_start, base_goal)

    test_arm_motion(pr2, left_joints, arm_goal)
    test_arm_control(pr2, left_joints, arm_start)

    input('Finish?')
    p.disconnect()

if __name__ == '__main__':
    main()