#!/usr/bin/env python

from __future__ import print_function

import os
import numpy as np
from pybullet_tools.kuka_kr6r900_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_ik_fn, get_free_motion_gen, get_holding_motion_gen

from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    Pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_for_interrupt, disconnect, user_input, update_state, disable_real_time, \
    joints_from_names, get_link_pose, link_from_name, set_joint_positions, get_joint_positions,\
    end_effector_from_body, approach_from_grasp, load_pybullet, HideOutput, dump_body, pairwise_collision

def plan(robot, block, fixed, teleport):
    grasp_gen = get_grasp_gen(robot, 'top')

    ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport)
    free_motion_fn = get_free_motion_gen(robot, fixed=([block] + fixed), teleport=teleport)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=fixed, teleport=teleport)

    pose0 = BodyPose(block)
    conf0 = BodyConf(robot)
    saved_world = WorldSaver()

    for i, (grasp,) in enumerate(grasp_gen(block)):
        saved_world.restore()
        result1 = ik_fn(block, pose0, grasp)

        if result1 is None:
            print("grasp %d - ik for grasp failed!" % i)
            continue
        conf1, path2 = result1

        # return Command(path2.body_paths)

        pose0.assign()
        result2 = free_motion_fn(conf0, conf1)
        if result2 is None:
            print("free motion failed!")
            continue
        path1, = result2

        result3 = holding_motion_fn(conf1, conf0, block, grasp)
        if result3 is None:
            print("holding motion failed!")
            continue
        path3, = result3

        return Command(path1.body_paths +
                        path2.body_paths +
                        path3.body_paths)
    return None

def test_ikfast_grasp_block(robot, block, fixed, process='grasp'):
    grasp_gen = get_grasp_gen(robot, 'top')

    # ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport)

    import pybullet_tools.kuka_kr6r900_ik.ik as ikfast
    arm_joints = joints_from_names(robot, ikfast.KUKA_KR6R900_GROUPS['arm_joints'])

    pose_world_from_block = BodyPose(block)
    conf0 = BodyConf(robot)
    saved_world = WorldSaver()

    obstacles = fixed

    for i, (grasp,) in enumerate(grasp_gen(block)):
        saved_world.restore()

        gripper_pose = end_effector_from_body(pose_world_from_block.pose, grasp.grasp_pose)
        approach_pose = approach_from_grasp(grasp.approach_pose, gripper_pose)

        if process == 'grasp':
            gen = ikfast.get_ik_generator(robot, gripper_pose)

        if process == 'approach':
            gen = ikfast.get_ik_generator(robot, approach_pose)

        solutions = next(gen)
        print("\nGrasp %d - sol number:%d" % (i, len(solutions)))
        for q in solutions:
            print(q)
            set_joint_positions(robot, arm_joints, q)

            for b in obstacles:
                if pairwise_collision(robot, b):
                    print("collision with obj %d" % b)

            user_input('step?')

def test_ikfast(robot):
    '''
    An ikfast test function, print and render all the ikfast joint sols
    by 'ctrl-C'
    Modified from test_pr2_motion.py
    :param robot: loaded URDF robot model
    '''

    from pybullet_tools.kuka_kr6r900_ik.ik \
        import forward_kinematics, get_tool_pose, get_ik_generator, KUKA_KR6R900_GROUPS

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

    root_directory = os.path.dirname(os.path.abspath(__file__))

    kuka_urdf = "../models/kuka_kr6r900_description/framefab_kr6_r900_support/urdf/kr6_r900_mit_suction_gripper.urdf"

    with HideOutput():
        robot = load_pybullet(os.path.join(root_directory, kuka_urdf), fixed_base=True)
        floor = load_model('models/short_floor.urdf')
        block = load_model(BLOCK_URDF, fixed_base=False)

    set_pose(block, Pose(Point(x=0.55, y=0, z=stable_z(block, floor)+0)))

    # TODO: need to do this check to bypass the collision between robot's base and the floor
    # Is there a way to disable the check between these two bodies?
    set_pose(floor, Pose(Point(z=-0.005)))

    set_default_camera()
    dump_world()

    print("-------------------------")
    print("fixed obj, floor id: %d" % floor)
    print("fixed obj, block id: %d" % block)

    from pybullet_tools.kuka_kr6r900_ik.ik import KUKA_KR6R900_GROUPS

    arm_joints = joints_from_names(robot, KUKA_KR6R900_GROUPS['arm_joints'])
    arm_start = [0.08, -1.57, 1.74, 0.08, 0.17, -0.08]

    set_joint_positions(robot, arm_joints, arm_start)

    # test_ikfast(robot)
    # test_ikfast_grasp_block(robot, block, fixed=[floor], process='approach')
    # test_ikfast_grasp_block(robot, block, fixed=[floor], process='grasp')

    saved_world = WorldSaver()
    command = plan(robot, block, fixed=[floor], teleport=False)
    if command is None:
        print('Unable to find a plan!')
        # return

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