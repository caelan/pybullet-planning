import importlib
import time
import numpy as np
import sys
import os
sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))

from itertools import islice

from .utils import compute_inverse_kinematics
from ..utils import get_link_pose, link_from_name, multiply, invert, get_link_ancestors, \
    parent_joint_from_link, parent_link_from_joint, prune_fixed_joints, joints_from_names, INF, get_difference_fn, \
    get_joint_positions, get_min_limits, get_max_limits, interval_generator, elapsed_time, randomize, violates_limits, \
    get_length

# TODO: forward kinematics


def import_ikfast(ikfast_info):
    # https://stackoverflow.com/questions/67631/how-to-import-a-module-given-the-full-path
    #print(sys.modules['__main__'].__file__)
    #return importlib.import_module('pybullet_tools.ikfast.{}'.format(ikfast_info.module_name), package=None)
    #return importlib.import_module('{}'.format(ikfast_info.module_name), package='pybullet_tools.ikfast')
    return importlib.import_module('ikfast.{}'.format(ikfast_info.module_name), package=None)


def is_ik_compiled(ikfast_info):
    try:
        import_ikfast(ikfast_info)
        return True
    except ImportError:
        return False


def get_base_from_ee(robot, ikfast_info, tool_link, world_from_target):
    world_from_base = get_link_pose(robot, link_from_name(robot, ikfast_info.base_link))
    world_from_ee = get_link_pose(robot, link_from_name(robot, ikfast_info.ee_link))
    world_from_tool = get_link_pose(robot, tool_link)
    tool_from_ee = multiply(invert(world_from_tool), world_from_ee)
    base_from_ee = multiply(invert(world_from_base), world_from_target, tool_from_ee)
    return base_from_ee


def get_ordered_ancestors(robot, link):
    #return prune_fixed_joints(robot, get_link_ancestors(robot, link)[1:] + [link])
    return get_link_ancestors(robot, link)[1:] + [link]


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
    free_deltas = np.array([0. if joint in fixed_joints else max_distance for joint in free_joints])
    lower_limits = np.maximum(get_min_limits(robot, free_joints), current_positions - free_deltas)
    upper_limits = np.minimum(get_max_limits(robot, free_joints), current_positions + free_deltas)
    generator = interval_generator(lower_limits, upper_limits)
    if max_attempts < INF:
        generator = islice(generator, max_attempts)
    start_time = time.time()
    for free_positions in generator:
        if max_time < elapsed_time(start_time):
            break
        for conf in randomize(compute_inverse_kinematics(ikfast.get_ik, base_from_ee, free_positions)):
            difference = difference_fn(current_conf, conf)
            if not violates_limits(robot, ik_joints, conf) and get_length(difference, norm=norm):
                #set_joint_positions(robot, ik_joints, conf)
                yield conf


def closest_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target,
                               max_candidates=INF, norm=INF, **kwargs):
    start_time = time.time()
    ik_joints = get_ik_joints(robot, ikfast_info, tool_link)
    current_conf = get_joint_positions(robot, ik_joints)
    generator = ikfast_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target, norm=norm, **kwargs)
    if max_candidates < INF:
        generator = islice(generator, max_candidates)
    solutions = list(generator)
    #print('Identified {} IK solutions in {:.3f} seconds'.format(len(solutions), elapsed_time(start_time)))
    # TODO: relative to joint limits
    difference_fn = get_difference_fn(robot, ik_joints)
    #distance_fn = get_distance_fn(robot, ik_joints)
    #set_joint_positions(robot, ik_joints, closest_conf)
    return iter(sorted(solutions, key=lambda q: get_length(difference_fn(q, current_conf), norm=norm)))