#!/usr/bin/env python

import argparse
import pybullet as p
import random

from pybullet_tools.pr2_utils import set_arm_conf, get_other_arm, arm_conf, REST_LEFT_ARM, \
    get_carry_conf, get_gripper_link, GET_GRASPS, IR_FILENAME, get_database_file
from pybullet_tools.utils import create_box, disconnect, add_data_path, connect, get_movable_joints, get_joint_positions, \
    sample_placement, set_pose, multiply, invert, set_joint_positions, pairwise_collision, inverse_kinematics, \
    get_link_pose, get_pose, get_body_name, write_pickle, uniform_pose_generator, set_base_values


def create_inverse_reachability(robot, body, table, arm, grasp_type, num_samples=500):
    link = get_gripper_link(robot, arm)
    movable_joints = get_movable_joints(robot)
    default_conf = get_joint_positions(robot, movable_joints)
    gripper_from_base_list = []
    grasps = GET_GRASPS[grasp_type](body)

    while len(gripper_from_base_list) < num_samples:
        box_pose = sample_placement(body, table)
        set_pose(body, box_pose)
        grasp_pose = random.choice(grasps)
        gripper_pose = multiply(box_pose, invert(grasp_pose))
        set_joint_positions(robot, movable_joints, default_conf)
        base_conf = next(uniform_pose_generator(robot, gripper_pose))
        set_base_values(robot, base_conf)
        if pairwise_collision(robot, table):
            continue
        conf = inverse_kinematics(robot, link, gripper_pose)
        if (conf is None) or pairwise_collision(robot, table):
            continue
        gripper_from_base = multiply(invert(get_link_pose(robot, link)), get_pose(robot))
        gripper_from_base_list.append(gripper_from_base)
        print('{} / {}'.format(len(gripper_from_base_list), num_samples))

    filename = IR_FILENAME.format(grasp_type, arm)
    path = get_database_file(filename)
    data = {
        'filename': filename,
        'robot': get_body_name(robot),
        'grasp_type': grasp_type,
        'arg': arm,
        'carry_conf': get_carry_conf(arm, grasp_type),
        'gripper_link': link,
        'gripper_from_base': gripper_from_base_list,
    }
    write_pickle(path, data)
    return path

def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-arm', required=True)
    parser.add_argument('-grasp', required=True)
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    arm = args.arm
    other_arm = get_other_arm(arm)
    grasp_type = args.grasp

    connect(use_gui=args.viewer)
    add_data_path()

    robot = p.loadURDF("models/pr2_description/pr2_fixed_torso.urdf")
    set_arm_conf(robot, arm, get_carry_conf(arm, grasp_type))
    set_arm_conf(robot, other_arm, arm_conf(other_arm, REST_LEFT_ARM))

    #plane = p.loadURDF("plane.urdf")
    table = p.loadURDF("table/table.urdf", 0, 0, 0, 0, 0, 0.707107, 0.707107)
    box = create_box(.07, .05, .15)

    create_inverse_reachability(robot, box, table, arm=arm, grasp_type=grasp_type)
    disconnect()

if __name__ == '__main__':
    main()
