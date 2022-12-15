#!/usr/bin/env python

from __future__ import print_function

import argparse
import numpy as np

from pybullet_tools.pr2_utils import PR2_URDF, DRAKE_PR2_URDF, \
    SIDE_HOLDING_LEFT_ARM, PR2_GROUPS, open_arm, REST_LEFT_ARM, rightarm_from_leftarm, PR2_TOOL_FRAMES, LEFT_ARM
from pybullet_tools.tracik import IKSolver
from pybullet_tools.utils import set_joint_positions, add_data_path, connect, dump_body, load_model, joints_from_names, \
    disconnect, HideOutput, load_pybullet, base_aligned_z, Point, set_point, get_aabb, \
    FLOOR_URDF, wait_unlocked, multiply, Pose, Point


def main(arm=LEFT_ARM):
    np.set_printoptions(precision=3)

    parser = argparse.ArgumentParser()
    parser.add_argument('--first_joint', default=None, choices=[None, 'x', 'y', 'theta', 'torso_lift_joint', 'l_shoulder_pan_joint'])
    parser.add_argument('--drake', action='store_true')
    args = parser.parse_args()
    print('Args:', args)

    connect(use_gui=True)
    add_data_path()
    floor = load_pybullet(FLOOR_URDF)

    urdf_path = DRAKE_PR2_URDF if args.drake else PR2_URDF
    with HideOutput():
        robot = load_model(urdf_path, fixed_base=True) # TODO: suppress warnings?
    dump_body(robot)

    z = base_aligned_z(robot)
    #z = stable_z_on_aabb(robot, AABB(np.zeros(3), np.zeros(3)))
    print('Base z: {:.3f}'.format(z))
    set_point(robot, Point(z=z))
    open_arm(robot, arm)

    group_confs = {
        'left_arm': SIDE_HOLDING_LEFT_ARM, # TOP_HOLDING_LEFT_ARM | SIDE_HOLDING_LEFT_ARM
        'right_arm': rightarm_from_leftarm(REST_LEFT_ARM),
        'torso': [0.2],
    }
    for group, conf in group_confs.items():
        joints = joints_from_names(robot, PR2_GROUPS[group])
        set_joint_positions(robot, joints, conf)

    ########################################

    tool_link = PR2_TOOL_FRAMES[LEFT_ARM]
    ik_solver = IKSolver(robot, tool_link=tool_link, first_joint=args.first_joint)
    print(ik_solver)

    tool_pose = ik_solver.get_tool_pose()
    #target_pose = tool_pose
    target_pose = multiply(Pose(point=Point(z=0.05)), tool_pose)
    ik_solver.draw_pose(target_pose)
    print('Target pose:', target_pose)
    print('Lower:', ik_solver.lower_limits)
    print('Upper:', ik_solver.upper_limits)
    print('Initial conf:', np.array(ik_solver.get_conf()))
    wait_unlocked('Continue')

    for i, conf in enumerate(ik_solver.generate(tool_pose)):
        print('{}) Solution: {}'.format(i, np.array(conf)))
        if conf is not None:
            ik_solver.set_conf(conf)
        wait_unlocked('Continue')

    wait_unlocked('Finish?')
    disconnect()

if __name__ == '__main__':
    main()
