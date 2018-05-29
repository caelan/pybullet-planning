#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import numpy as np

from pr2_utils import TORSO_JOINT_NAME, HEAD_LINK_NAME, \
    PR2_GROUPS, get_cone_mesh, \
    REST_LEFT_ARM, rightarm_from_leftarm, inverse_visibility, get_detection_cone
from utils import joint_from_name, set_joint_position, \
    set_joint_positions, connect, wait_for_interrupt, get_link_pose, link_from_name, set_point, set_pose, \
    dump_body, load_model, create_mesh, unit_from_theta, \
    point_from_pose, get_pose, joints_from_names, BLOCK_URDF, remove_body

def visible_base_generator(robot, target_point, distance_range):
    #base_from_table = point_from_pose(get_pose(robot))[:2]
    while True:
        base_from_table = unit_from_theta(np.random.uniform(0, 2 * np.pi))
        look_distance = np.random.uniform(*distance_range)
        base_xy = target_point[:2] - look_distance * base_from_table
        base_theta = np.math.atan2(base_from_table[1], base_from_table[0])
        base_q = np.append(base_xy, base_theta)
        yield base_q

def main():
    connect(use_gui=True)

    pr2 = load_model("models/drake/pr2_description/urdf/pr2_simplified.urdf")
    set_joint_positions(pr2, joints_from_names(pr2, PR2_GROUPS['left_arm']), REST_LEFT_ARM)
    set_joint_positions(pr2, joints_from_names(pr2, PR2_GROUPS['right_arm']), rightarm_from_leftarm(REST_LEFT_ARM))
    set_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT_NAME), 0.2)
    dump_body(pr2)

    block = load_model(BLOCK_URDF, fixed_base=False)
    set_point(block, [2, 0.5, 1])
    target_point = point_from_pose(get_pose(block))

    # head_link = link_from_name(pr2, HEAD_LINK)
    head_joints = joints_from_names(pr2, PR2_GROUPS['head'])
    head_link = head_joints[-1]

    #max_detect_distance = 2.5
    max_register_distance = 1.0
    distance_range = (max_register_distance/2, max_register_distance)
    base_generator = visible_base_generator(pr2, target_point, distance_range)

    for i in range(5):
        base_conf = next(base_generator)
        set_joint_positions(pr2, joints_from_names(pr2, PR2_GROUPS['base']), base_conf)

        p.addUserDebugLine(point_from_pose(get_link_pose(pr2, head_link)),
                           target_point, lineColorRGB=(1, 0, 0))  # addUserDebugText
        p.addUserDebugLine(point_from_pose(get_link_pose(pr2, link_from_name(pr2, HEAD_LINK_NAME))),
                           target_point, lineColorRGB=(0, 0, 1))  # addUserDebugText

        # head_conf = sub_inverse_kinematics(pr2, head_joints[0], HEAD_LINK, )
        head_conf = inverse_visibility(pr2, target_point)
        set_joint_positions(pr2, head_joints, head_conf)

        head_pose = get_link_pose(pr2, link_from_name(pr2, HEAD_LINK_NAME))
        detect_cone = create_mesh(get_detection_cone(pr2, block), color=(0, 1, 0, 0.5))
        set_pose(detect_cone, head_pose)
        view_cone = create_mesh(get_cone_mesh(depth=2.5), color=(1, 0, 0, 0.25))
        set_pose(view_cone, get_link_pose(pr2, link_from_name(pr2, HEAD_LINK_NAME)))
        wait_for_interrupt()
        remove_body(detect_cone)
        remove_body(view_cone)

    p.disconnect()

if __name__ == '__main__':
    main()