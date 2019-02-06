#!/usr/bin/env python

from __future__ import print_function

import os
from pybullet_tools.kuka_primitives import BodyPose, BodyConf, Command, \
    get_free_motion_gen, get_holding_motion_gen, BodyPath, Attach
from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    Pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_for_interrupt, disconnect, user_input, update_state, disable_real_time, \
    load_pybullet
from pybullet_tools.utils import get_movable_joints, get_configuration, \
    set_joint_positions, add_fixed_constraint, enable_real_time, joint_controller, \
    enable_gravity, get_refine_fn, wait_for_duration, link_from_name, get_body_name, sample_placement, \
    end_effector_from_body, approach_from_grasp, plan_joint_motion, GraspInfo, INF, \
    inverse_kinematics, pairwise_collision, remove_fixed_constraint, Attachment, get_sample_fn, \
    step_simulation, refine_path, plan_direct_joint_motion, draw_pose, joints_from_names

from pybullet_tools.eth_rfl_utils import get_torso_arm_joints, get_grasp_gen
from pybullet_tools.ikfast.eth_rfl.ik import sample_tool_ik, get_tool_pose

USE_IKFAST = True
DEBUG_FAILURE = True
ENABLE_SELF_COLLISION = False

ARM = 'right' # 'left'

def get_ik_fn(robot, fixed=[], teleport=False, num_attempts=10, self_collisions=True):
    # movable_joints = get_movable_joints(robot)
    torso_arm = get_torso_arm_joints(robot, ARM)
    sample_fn = get_sample_fn(robot, torso_arm)

    def fn(body, pose, grasp):
        obstacles = [body] + fixed
        gripper_pose = end_effector_from_body(pose.pose, grasp.grasp_pose)
        approach_pose = approach_from_grasp(grasp.approach_pose, gripper_pose)

        draw_pose(get_tool_pose(robot, ARM), length=0.04)
        draw_pose(approach_pose, length=0.04)
        draw_pose(gripper_pose, length=0.04)
        # print(movable_joints)
        # print(torso_arm)
        # wait_for_interrupt()

        for _ in range(num_attempts):
            if USE_IKFAST:
                q_approach = sample_tool_ik(robot, ARM, approach_pose)
            else:
                set_joint_positions(robot, torso_arm, sample_fn()) # Random seed
                q_approach = inverse_kinematics(robot, grasp.link, approach_pose)

            if (q_approach is None) or any(pairwise_collision(robot, b) for b in obstacles):
                print('- ik for approaching fails!')
                continue

            # set_joint_positions(robot, movable_joints, q_approach) # Random seed
            conf = BodyConf(robot, q_approach, torso_arm)

            if USE_IKFAST:
                q_grasp = sample_tool_ik(robot, ARM, gripper_pose, nearby_conf=q_approach)
            else:
                conf.assign()
                q_grasp = inverse_kinematics(robot, grasp.link, gripper_pose)

            if (q_grasp is None) or any(pairwise_collision(robot, b) for b in obstacles):
                print('- ik for grasp fails!')
                continue

            if teleport:
                path = [q_approach, q_grasp]
            else:
                conf.assign()
                #direction, _ = grasp.approach_pose
                #path = workspace_trajectory(robot, grasp.link, point_from_pose(approach_pose), -direction,
                #                                   quat_from_pose(approach_pose))
                path = plan_direct_joint_motion(robot, conf.joints, q_grasp, obstacles=obstacles, \
                                                self_collisions=self_collisions)
                if path is None:
                    if DEBUG_FAILURE: print('Approach motion failed')
                    continue
            command = Command([BodyPath(robot, path, joints=torso_arm),
                               Attach(body, robot, grasp.link),
                               BodyPath(robot, path[::-1], joints=torso_arm, attachments=[grasp])])
            return (conf, command)
            # TODO: holding collisions
        return None
    return fn

def plan(robot, block, fixed, teleport):
    grasp_gen = get_grasp_gen(robot, ARM, 'bottom') #'top'
    ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport, self_collisions=ENABLE_SELF_COLLISION)
    free_motion_fn = get_free_motion_gen(robot, fixed=([block] + fixed), teleport=teleport, self_collisions=ENABLE_SELF_COLLISION)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=fixed, teleport=teleport, self_collisions=ENABLE_SELF_COLLISION)

    torso_arm = get_torso_arm_joints(robot, ARM)

    pose0 = BodyPose(block)
    conf0 = BodyConf(robot, joints=torso_arm)
    saved_world = WorldSaver()
    for grasp, in grasp_gen(block):
        saved_world.restore()
        result1 = ik_fn(block, pose0, grasp)
        if result1 is None:
            print('ik fn fails!')
            # continue
        conf1, path2 = result1
        pose0.assign()
        result2 = free_motion_fn(conf0, conf1)
        if result2 is None:
            print("free motion plan fails!")
            continue
        path1, = result2
        result3 = holding_motion_fn(conf1, conf0, block, grasp)
        if result3 is None:
            print('holding motion fails!')
            continue
        path3, = result3
        return Command(path1.body_paths +
                          path2.body_paths +
                          path3.body_paths)
    return None


def main(display='execute'): # control | execute | step
    root_directory = os.path.dirname(os.path.abspath(__file__))

    connect(use_gui=True)
    disable_real_time()

    ETH_RFL_URDF = "../models/eth_rfl/urdf/eth_rfl.urdf"
    robot = load_pybullet(os.path.join(root_directory, ETH_RFL_URDF), fixed_base=True)
    # floor = load_model('models/short_floor.urdf')
    block = load_model(BLOCK_URDF, fixed_base=False)

    block_x = -0.2
    if ARM == 'right':
        block_y = 1
    else:
        block_y = 13.5

    # set_pose(floor, Pose(Point(x=floor_x, y=1, z=1.3)))
    # set_pose(block, Pose(Point(x=floor_x, y=0.6, z=stable_z(block, floor))))
    set_pose(block, Pose(Point(x=block_x, y=block_y, z=3.5)))
    # set_default_camera()
    dump_world()

    wait_for_interrupt()

    saved_world = WorldSaver()
    command = plan(robot, block, fixed=[], teleport=False) # fixed=[floor],
    if (command is None) or (display is None):
        print('Unable to find a plan!')
        print('Quit?')
        wait_for_interrupt()
        disconnect()
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