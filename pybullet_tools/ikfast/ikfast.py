from __future__ import print_function

import importlib
import time
import numpy as np
import sys
import os

PARENT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
sys.path.append(PARENT_DIR)

from itertools import islice, chain

from .utils import compute_inverse_kinematics, compute_forward_kinematics
from ..utils import get_link_pose, link_from_name, multiply, invert, parent_joint_from_link, parent_link_from_joint, \
    prune_fixed_joints, joints_from_names, INF, get_difference_fn, \
    get_joint_positions, get_min_limits, get_max_limits, interval_generator, elapsed_time, randomize, violates_limits, \
    get_length, get_relative_pose, set_joint_positions, get_pose_distance, ConfSaver, \
    sub_inverse_kinematics, set_configuration, wait_for_user, multiple_sub_inverse_kinematics, get_ordered_ancestors

SETUP_FILENAME = 'setup.py'


def get_module_name(ikfast_info):
    return 'ikfast.{}'.format(ikfast_info.module_name)


def import_ikfast(ikfast_info):
    # https://stackoverflow.com/questions/67631/how-to-import-a-module-given-the-full-path
    #print(sys.modules['__main__'].__file__)
    #return importlib.import_module('pybullet_tools.ikfast.{}'.format(ikfast_info.module_name), package=None)
    #return importlib.import_module('{}'.format(ikfast_info.module_name), package='pybullet_tools.ikfast')
    return importlib.import_module(get_module_name(ikfast_info), package=None)


def is_ik_compiled(ikfast_info):
    try:
        import_ikfast(ikfast_info)
        return True
    except ImportError:
        return False


def check_ik_solver(ikfast_info):
    print(ikfast_info)
    try:
        import_ikfast(ikfast_info)
        print('Using IKFast for inverse kinematics')
    except ImportError as e:
        #traceback.print_exc()
        #print('\x1b[6;30;43m' + '{}, Using pybullet ik fn instead'.format(e) + '\x1b[0m')
        print(e)
        module_name = get_module_name(ikfast_info)
        ik_path = os.path.join(PARENT_DIR, '/'.join(module_name.split('.')[:-1]))
        build_path = os.path.join(ik_path, SETUP_FILENAME)
        print('Could not import IKFast module {}, please compile {} to use IKFast.'.format(
            module_name, build_path))
        print('$ (cd {}; ./{})'.format(ik_path, SETUP_FILENAME))
        print('Using PyBullet for inverse kinematics')
        return True
    else:
        return False

##################################################

def get_base_from_ee(robot, ikfast_info, tool_link, world_from_target):
    ee_link = link_from_name(robot, ikfast_info.ee_link)
    tool_from_ee = get_relative_pose(robot, ee_link, tool_link)
    world_from_base = get_link_pose(robot, link_from_name(robot, ikfast_info.base_link))
    return multiply(invert(world_from_base), world_from_target, tool_from_ee)


def get_ik_joints(robot, ikfast_info, tool_link):
    # Get joints between base and ee
    # Ensure no joints between ee and tool
    base_link = link_from_name(robot, ikfast_info.base_link)
    ee_link = link_from_name(robot, ikfast_info.ee_link)
    ee_ancestors = get_ordered_ancestors(robot, ee_link)
    tool_ancestors = get_ordered_ancestors(robot, tool_link)
    [first_joint] = [parent_joint_from_link(link) for link in tool_ancestors
                     if parent_link_from_joint(robot, parent_joint_from_link(link)) == base_link]
    assert prune_fixed_joints(robot, ee_ancestors) == prune_fixed_joints(robot, tool_ancestors)
    #assert base_link in ee_ancestors # base_link might be -1
    ik_joints = prune_fixed_joints(robot, ee_ancestors[ee_ancestors.index(first_joint):])
    free_joints = joints_from_names(robot, ikfast_info.free_joints)
    assert set(free_joints) <= set(ik_joints)
    assert len(ik_joints) == 6 + len(free_joints)
    return ik_joints

##################################################

def check_solution(robot, joints, conf, tool_link, target_pose, tolerance=1e-6):
    with ConfSaver(robot, joints=joints):
        set_joint_positions(robot, joints, conf)
        actual_pose = get_link_pose(robot, tool_link)
    pos_distance, ori_distance = get_pose_distance(target_pose, actual_pose)
    valid = (pos_distance <= tolerance) and (ori_distance <= tolerance)
    if not valid:
        print('IKFast warning! | Valid: {} | Position error: {:.3e} | Orientation error: {:.3e}'.format(
            valid, pos_distance, ori_distance))
    return valid


def ikfast_forward_kinematics(robot, ikfast_info, tool_link, conf=None, use_ikfast=True):
    # TODO: cleanup ./pr2/ik.py
    # from .ikLeft import leftFK
    # from .ikRight import rightFK
    # arm_fk = {'left': leftFK, 'right': rightFK}
    # fk_fn = arm_fk[arm]

    ik_joints = get_ik_joints(robot, ikfast_info, tool_link)
    if conf is None:
        conf = get_joint_positions(robot, ik_joints)

    if not use_ikfast:
        set_joint_positions(robot, ik_joints, conf)
        world_from_tool = get_link_pose(robot, tool_link)
        # print(world_from_tool)
        check_solution(robot, ik_joints, conf, tool_link, world_from_tool)
        return world_from_tool

    ikfast = import_ikfast(ikfast_info)
    base_from_ee = compute_forward_kinematics(ikfast.get_fk, conf)

    world_from_base = get_link_pose(robot, link_from_name(robot, ikfast_info.base_link))
    tool_from_ee = get_relative_pose(robot, link_from_name(robot, ikfast_info.ee_link), tool_link)
    world_from_tool = multiply(world_from_base, base_from_ee, invert(tool_from_ee))

    #print(world_from_tool)
    check_solution(robot, ik_joints, conf, tool_link, world_from_tool)
    return world_from_tool

##################################################

def ikfast_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target,
                              fixed_joints=[], max_attempts=INF, max_time=INF,
                              norm=INF, max_distance=INF, **kwargs):
    assert (max_attempts < INF) or (max_time < INF)
    if max_distance is None:
        max_distance = INF
    #assert is_ik_compiled(ikfast_info)
    ikfast = import_ikfast(ikfast_info)
    ik_joints = get_ik_joints(robot, ikfast_info, tool_link)
    free_joints = joints_from_names(robot, ikfast_info.free_joints)
    base_from_ee = get_base_from_ee(robot, ikfast_info, tool_link, world_from_target)
    difference_fn = get_difference_fn(robot, ik_joints)
    current_conf = get_joint_positions(robot, ik_joints)
    current_positions = get_joint_positions(robot, free_joints)

    # TODO: handle circular joints
    # TODO: use norm=INF to limit the search for free values
    free_deltas = np.array([0. if joint in fixed_joints else max_distance for joint in free_joints])
    lower_limits = np.maximum(get_min_limits(robot, free_joints), current_positions - free_deltas)
    upper_limits = np.minimum(get_max_limits(robot, free_joints), current_positions + free_deltas)
    generator = chain([current_positions], # TODO: sample from a truncated Gaussian nearby
                      interval_generator(lower_limits, upper_limits))
    if max_attempts < INF:
        generator = islice(generator, max_attempts)
    start_time = time.time()
    for free_positions in generator:
        if max_time < elapsed_time(start_time):
            break
        for conf in randomize(compute_inverse_kinematics(ikfast.get_ik, base_from_ee, free_positions)):
            #solution(robot, ik_joints, conf, tool_link, world_from_target)
            difference = difference_fn(current_conf, conf)
            if not violates_limits(robot, ik_joints, conf) and (get_length(difference, norm=norm) <= max_distance):
                #set_joint_positions(robot, ik_joints, conf)
                yield conf


def closest_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target,
                               max_candidates=INF, norm=INF, verbose=True, **kwargs):
    start_time = time.time()
    ik_joints = get_ik_joints(robot, ikfast_info, tool_link)
    current_conf = get_joint_positions(robot, ik_joints)
    generator = ikfast_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target, norm=norm, **kwargs)
    if max_candidates < INF:
        generator = islice(generator, max_candidates)
    solutions = list(generator)
    # TODO: relative to joint limits
    difference_fn = get_difference_fn(robot, ik_joints) # get_distance_fn
    solutions = sorted(solutions, key=lambda q: get_length(difference_fn(q, current_conf), norm=norm))
    if verbose:
        min_distance = min([INF] + [get_length(difference_fn(q, current_conf), norm=norm) for q in solutions])
        print('Identified {} IK solutions with minimum distance of {:.3f} in {:.3f} seconds'.format(
            len(solutions), min_distance, elapsed_time(start_time)))
    return iter(solutions)


##################################################


def pybullet_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target, fixed_joints=[], **kwargs):
    ik_joints = get_ik_joints(robot, ikfast_info, tool_link)
    free_joints = [joint for joint in ik_joints if joint not in fixed_joints]
    assert free_joints
    first_joint = free_joints[0]
    # conf = sub_inverse_kinematics(robot, first_joint, tool_link, world_from_target, **kwargs)
    # solutions = [] if conf is None else [conf]
    # TODO: sample multiple solutions and return
    solutions = multiple_sub_inverse_kinematics(robot, first_joint, tool_link, world_from_target,
                                                max_attempts=1, first_close=True, **kwargs)
    for solution in solutions: # TODO: sort by distance
        set_configuration(robot, solution)
        yield get_joint_positions(robot, ik_joints)


def either_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target, fixed_joints=[],
                              use_pybullet=False, **kwargs):
    if not use_pybullet and is_ik_compiled(ikfast_info):
        return closest_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target, fixed_joints=fixed_joints, **kwargs)
    return pybullet_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target, fixed_joints=[])
