#!/usr/bin/env python

import argparse
import random
import time

from pybullet_tools.pr2_utils import set_arm_conf, get_other_arm, arm_conf, REST_LEFT_ARM, \
    get_carry_conf, get_gripper_link, GET_GRASPS, IR_FILENAME, get_database_file, DRAKE_PR2_URDF, \
    set_group_conf, get_group_conf, get_base_pose
from pybullet_tools.utils import create_box, disconnect, add_data_path, connect, get_movable_joints, get_joint_positions, \
    sample_placement, set_pose, multiply, invert, set_joint_positions, pairwise_collision, inverse_kinematics, \
    get_link_pose, get_body_name, write_pickle, uniform_pose_generator, set_base_values, \
    load_pybullet, HideOutput, wait_for_user, draw_point, point_from_pose, has_gui, elapsed_time, \
    sub_inverse_kinematics, BodySaver
from pybullet_tools.pr2_problems import create_table
from pybullet_tools.ikfast.pr2.ik import pr2_inverse_kinematics, is_ik_compiled
from pybullet_tools.ikfast.utils import USE_CURRENT
from pybullet_tools.pr2_primitives import get_stable_gen, get_grasp_gen, get_ik_ir_gen

def save_inverse_reachability(robot, arm, grasp_type, tool_link, gripper_from_base_list):
    # TODO: store value of torso and roll joint for the IK database. Sample the roll joint.
    # TODO: hash the pr2 urdf as well
    filename = IR_FILENAME.format(grasp_type, arm)
    path = get_database_file(filename)
    data = {
        'filename': filename,
        'robot': get_body_name(robot),
        'grasp_type': grasp_type,
        'arm': arm,
        'torso': get_group_conf(robot, 'torso'),
        'carry_conf': get_carry_conf(arm, grasp_type),
        'tool_link': tool_link,
        'ikfast': is_ik_compiled(),
        'gripper_from_base': gripper_from_base_list,
    }
    write_pickle(path, data)

    if has_gui():
        handles = []
        for gripper_from_base in gripper_from_base_list:
            handles.extend(draw_point(point_from_pose(gripper_from_base), color=(1, 0, 0)))
        wait_for_user()
    return path

#######################################################

def create_inverse_reachability(robot, body, table, arm, grasp_type, max_attempts=500, num_samples=500):
    tool_link = get_gripper_link(robot, arm)
    robot_saver = BodySaver(robot)
    gripper_from_base_list = []
    grasps = GET_GRASPS[grasp_type](body)

    start_time = time.time()
    while len(gripper_from_base_list) < num_samples:
        box_pose = sample_placement(body, table)
        set_pose(body, box_pose)
        grasp_pose = random.choice(grasps)
        gripper_pose = multiply(box_pose, invert(grasp_pose))
        for attempt in range(max_attempts):
            robot_saver.restore()
            base_conf = next(uniform_pose_generator(robot, gripper_pose)) #, reachable_range=(0., 1.)))
            #set_base_values(robot, base_conf)
            set_group_conf(robot, 'base', base_conf)
            if pairwise_collision(robot, table):
                continue
            grasp_conf = pr2_inverse_kinematics(robot, arm, gripper_pose) #, nearby_conf=USE_CURRENT)
            #conf = inverse_kinematics(robot, link, gripper_pose)
            if (grasp_conf is None) or pairwise_collision(robot, table):
                continue
            gripper_from_base = multiply(invert(get_link_pose(robot, tool_link)), get_base_pose(robot))
            #wait_for_user()
            gripper_from_base_list.append(gripper_from_base)
            print('{} / {} | {} attempts | [{:.3f}]'.format(
                len(gripper_from_base_list), num_samples, attempt, elapsed_time(start_time)))
            if has_gui():
                wait_for_user()
            break
        else:
            print('Failed to find a kinematic solution after {} attempts'.format(max_attempts))
    return save_inverse_reachability(robot, arm, grasp_type, tool_link, gripper_from_base_list)

#######################################################

class MockProblem(object):
    def __init__(self, robot, fixed=[], grasp_types=[]):
        self.robot = robot
        self.fixed = fixed
        self.grasp_types = grasp_types

def create_inverse_reachability2(robot, body, table, arm, grasp_type, max_attempts=500, num_samples=500):
    tool_link = get_gripper_link(robot, arm)
    problem = MockProblem(robot, fixed=[table], grasp_types=[grasp_type])
    placement_gen_fn = get_stable_gen(problem)
    grasp_gen_fn = get_grasp_gen(problem, collisions=True)
    ik_ir_fn = get_ik_ir_gen(problem, max_attempts=max_attempts, learned=False, teleport=True)
    placement_gen = placement_gen_fn(body, table)
    grasps = list(grasp_gen_fn(body))
    print('Grasps:', len(grasps))

    # TODO: sample the torso height
    # TODO: consider IK with respect to the torso frame
    start_time = time.time()
    gripper_from_base_list = []
    while len(gripper_from_base_list) < num_samples:
        [(p,)] = next(placement_gen)
        (g,) = random.choice(grasps)
        output = next(ik_ir_fn(arm, body, p, g), None)
        if output is None:
            print('Failed to find a solution after {} attempts'.format(max_attempts))
        else:
            (_, ac) = output
            [at,] = ac.commands
            at.path[-1].assign()
            gripper_from_base = multiply(invert(get_link_pose(robot, tool_link)), get_base_pose(robot))
            gripper_from_base_list.append(gripper_from_base)
            print('{} / {} [{:.3f}]'.format(
                len(gripper_from_base_list), num_samples, elapsed_time(start_time)))
            if has_gui():
                wait_for_user()
    return save_inverse_reachability(robot, arm, grasp_type, tool_link, gripper_from_base_list)

#######################################################

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

    with HideOutput():
        robot = load_pybullet(DRAKE_PR2_URDF)
    set_group_conf(robot, 'torso', [0.2])
    set_arm_conf(robot, arm, get_carry_conf(arm, grasp_type))
    set_arm_conf(robot, other_arm, arm_conf(other_arm, REST_LEFT_ARM))

    #plane = p.loadURDF("plane.urdf")
    #table = p.loadURDF("table/table.urdf", 0, 0, 0, 0, 0, 0.707107, 0.707107)
    table = create_table()
    box = create_box(.07, .07, .14)

    #create_inverse_reachability(robot, box, table, arm=arm, grasp_type=grasp_type)
    create_inverse_reachability2(robot, box, table, arm=arm, grasp_type=grasp_type)
    disconnect()

if __name__ == '__main__':
    main()
