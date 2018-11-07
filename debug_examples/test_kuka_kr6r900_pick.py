#!/usr/bin/env python

from __future__ import print_function

from pybullet_tools.kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_ik_fn, get_free_motion_gen, get_holding_motion_gen

from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    Pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_for_interrupt, disconnect, DRAKE_IIWA_URDF, user_input, update_state, disable_real_time

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

# def test_ikfast(pr2):
#     from pybullet_tools.kuka_kr6r900_ik.ik import forward_kinematics, inverse_kinematics, get_tool_pose#, get_ik_generator
#
#     left_joints = joints_from_names(pr2, PR2_GROUPS['left_arm'])
#     #right_joints = joints_from_names(pr2, PR2_GROUPS['right_arm'])
#     torso_joints = joints_from_names(pr2, PR2_GROUPS['torso'])
#     torso_left = torso_joints + left_joints
#     print(get_link_pose(pr2, link_from_name(pr2, 'l_gripper_tool_frame')))
#     print(forward_kinematics('left', get_joint_positions(pr2, torso_left)))
#     print(get_tool_pose(pr2, 'left'))
#
#     arm = 'left'
#     pose = get_tool_pose(pr2, arm)
#     generator = get_ik_generator(pr2, arm, pose, torso_limits=False)
#     for i in range(100):
#         solutions = next(generator)
#         print(i, len(solutions))
#         for q in solutions:
#             set_joint_positions(pr2, torso_left, q)
#             wait_for_interrupt()

def main(display='execute'): # control | execute | step
    connect(use_gui=True)
    disable_real_time()

    KUKA_URDF = "models/kuka_kr6r900_description/framefab_kr6r900_support/urdf/kuka_kr6_r900.urdf"

    robot = load_model(KUKA_URDF) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
    floor = load_model('models/short_floor.urdf')
    block = load_model(BLOCK_URDF, fixed_base=False)
    set_pose(block, Pose(Point(y=0.5, z=stable_z(block, floor))))
    set_default_camera()
    dump_world()

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