#!/usr/bin/env python

from __future__ import print_function

import pybullet as p

from pybullet_tools.utils import add_data_path, connect, dump_body, load_model, disconnect, wait_for_user, \
    get_movable_joints, get_sample_fn, set_joint_positions, get_joint_name, joints_from_names, get_configuration, \
    LockRenderer, link_from_name, get_link_pose, INF, multiply, Pose, Point, interpolate_poses, HideOutput

from itertools import islice

from pybullet_tools.ikfast.franka_panda.ik import ikfast_inverse_kinematics, PANDA_INFO, closest_inverse_kinematics


#FRANKA_URDF = "models/franka_description/robots/panda_arm.urdf"
#FRANKA_URDF = "models/franka_description/robots/hand.urdf"
FRANKA_URDF = "models/franka_description/robots/panda_arm_hand.urdf"

#####################################

def main():
    connect(use_gui=True)
    add_data_path()

    plane = p.loadURDF("plane.urdf")
    with HideOutput():
        with LockRenderer():
            robot = load_model(FRANKA_URDF, fixed_base=True)
    dump_body(robot)
    print('Start?')
    wait_for_user()

    tool_link = link_from_name(robot, 'panda_hand')
    joints = get_movable_joints(robot)
    print('Joints', [get_joint_name(robot, joint) for joint in joints])
    sample_fn = get_sample_fn(robot, joints)
    for i in range(10):
        print('Iteration:', i)
        conf = sample_fn()
        set_joint_positions(robot, joints, conf)
        wait_for_user()

        tool_pose_world = get_link_pose(robot, tool_link)
        goal_pose_world = multiply(tool_pose_world, Pose(Point(z=-0.1)))
        for pose_word in interpolate_poses(tool_pose_world, goal_pose_world):
            conf = closest_inverse_kinematics(robot, PANDA_INFO, tool_link, pose_word, max_distance=0.05)
            if conf is None:
                print('Failure!')
                break
            set_joint_positions(robot, joints[:len(conf)], conf)
            wait_for_user()
            #for conf in islice(ikfast_inverse_kinematics(robot, PANDA_INFO, tool_link, pose_word, max_attempts=INF, max_distance=0.5), 1):
            #    set_joint_positions(robot, joints[:len(conf)], conf)
            #    wait_for_user()
    disconnect()

if __name__ == '__main__':
    main()
