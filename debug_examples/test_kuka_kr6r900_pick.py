#!/usr/bin/env python

from __future__ import print_function

import numpy as np
from pybullet_tools.kuka_kr6r900_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_ik_fn, get_free_motion_gen, get_holding_motion_gen

from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    Pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_for_interrupt, disconnect, DRAKE_IIWA_URDF, user_input, update_state, disable_real_time, \
    joints_from_names, get_link_pose, link_from_name, set_joint_positions, get_joint_positions

def plan(robot, block, fixed, teleport):
    grasp_gen = get_grasp_gen(robot, 'top')
    ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport)
    free_motion_fn = get_free_motion_gen(robot, fixed=([block] + fixed), teleport=teleport)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=fixed, teleport=teleport)

    pose0 = BodyPose(block)
    conf0 = BodyConf(robot)
    saved_world = WorldSaver()
    for grasp, in grasp_gen(block):
        saved_world.restore()
        result1 = ik_fn(block, pose0, grasp)
        if result1 is None:
            continue
        conf1, path2 = result1
        pose0.assign()
        result2 = free_motion_fn(conf0, conf1)
        if result2 is None:
            continue
        path1, = result2
        result3 = holding_motion_fn(conf1, conf0, block, grasp)
        if result3 is None:
            continue
        path3, = result3
        return Command(path1.body_paths +
                          path2.body_paths +
                          path3.body_paths)
    return None

def test_ikfast(robot):
    '''
    An ikfast test function, print and render all the ikfast joint sols
    by 'ctrl-C'
    Modified from test_pr2_motion.py
    :param robot: loaded URDF robot model
    '''

    from pybullet_tools.kuka_kr6r900_ik.ik \
        import forward_kinematics, inverse_kinematics, get_tool_pose, get_ik_generator, KUKA_KR6R900_GROUPS

    arm_joints = joints_from_names(robot, KUKA_KR6R900_GROUPS['arm_joints'])

    print(get_link_pose(robot, link_from_name(robot, KUKA_KR6R900_GROUPS['tool_link'])))
    print(forward_kinematics(get_joint_positions(robot, arm_joints)))
    print(get_tool_pose(robot))

    ee_pose = get_tool_pose(robot)
    generator = get_ik_generator(robot, ee_pose)

    for i in range(15):
        solutions = next(generator)
        print(i, len(solutions))
        for q in solutions:
            set_joint_positions(robot, arm_joints, q)
            wait_for_interrupt()

def main(display='execute'): # control | execute | step
    connect(use_gui=True)
    disable_real_time()

    KUKA_URDF = "models/kuka_kr6r900_description/framefab_kr6_r900_support/urdf/kuka_kr6_r900.urdf"
    robot = load_model(KUKA_URDF)

    floor = load_model('models/short_floor.urdf')
    block = load_model(BLOCK_URDF, fixed_base=False)
    set_pose(block, Pose(Point(y=0.5, z=stable_z(block, floor))))
    set_default_camera()
    dump_world()

    # from pybullet_tools.kuka_kr6r900_ik.ik import KUKA_KR6R900_GROUPS
    #
    # arm_joints = joints_from_names(robot, KUKA_KR6R900_GROUPS['arm_joints'])
    # arm_start = [np.pi/3, -np.pi * (1/2), -np.pi/2, np.pi*(1/6), 0, 0]
    # set_joint_positions(robot, arm_joints, arm_start)
    # test_ikfast(robot)

    saved_world = WorldSaver()
    command = plan(robot, block, fixed=[floor], teleport=False)
    if (command is None) or (display is None):
        print('Unable to find a plan!')
        return

    saved_world.restore()
    update_state()

    user_input('{}?'.format(display))
    if display == 'control':
        enable_gravity()
        command.control(real_time=False, dt=0)
    elif display == 'execute':
        command.refine(num_steps=10).execute(time_step=0.005)
    elif display == 'step':
        command.step()
    else:
        raise ValueError(display)

    print('Quit?')
    wait_for_interrupt()
    disconnect()

if __name__ == '__main__':
    main()