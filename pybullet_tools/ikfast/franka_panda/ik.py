import random
import numpy as np

from itertools import product, islice

from pybullet_tools.ikfast.utils import IKFastInfo
from ..utils import compute_inverse_kinematics
from ...utils import multiply, get_link_pose, link_from_name, invert, set_joint_positions, joints_from_names, \
    get_movable_joint_descendants, get_joint_limits, inf_generator, get_joint_position, randomize, violates_limits, \
    get_joint_positions, INF, get_difference_fn, get_distance_fn

PANDA_INFO = IKFastInfo(module_name='ikfast_panda_arm', base_link='panda_link0',
                        ee_link='panda_link8', free_joints=['panda_joint7'])

def import_ikfast(ikfast_info):
    # TODO: relative import
    import pybullet_tools.ikfast.franka_panda.ikfast_panda_arm as ikfast
    return ikfast
    #return importlib.import_module(ikfast_info.module_name, package=pybullet_tools.ikfast.franka_panda)

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

def get_ik_joints(robot, ikfast_info, tool_link):
    free_joints = joints_from_names(robot, ikfast_info.free_joints)
    base_descendants = get_movable_joint_descendants(robot, link_from_name(robot, ikfast_info.base_link))
    ee_descendants = get_movable_joint_descendants(robot, link_from_name(robot, ikfast_info.ee_link))
    assert ee_descendants == get_movable_joint_descendants(robot, tool_link)
    ik_joints = [joint for joint in base_descendants if joint not in ee_descendants]
    assert set(free_joints) <= set(ik_joints)
    assert len(ik_joints) == 6 + len(free_joints)
    return ik_joints

def ikfast_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target,
                              max_attempts=INF, max_distance=None, num=21):
    #assert is_ik_compiled(ikfast_info)
    ikfast = import_ikfast(ikfast_info)
    ik_joints = get_ik_joints(robot, ikfast_info, tool_link)
    free_joints = joints_from_names(robot, ikfast_info.free_joints)
    base_from_ee = get_base_from_ee(robot, ikfast_info, tool_link, world_from_target)
    difference_fn = get_difference_fn(robot, ik_joints)
    current_conf = get_joint_positions(robot, ik_joints)
    if max_distance is None:
        free_limits = [get_joint_limits(robot, joint) for joint in free_joints]
        generator = ([random.uniform(*pair) for pair in free_limits] for _ in inf_generator())
    else:
        deltas = list(np.linspace(-max_distance, +max_distance, endpoint=True, num=1+num))
        free_values = [[get_joint_position(robot, joint) + delta for delta in deltas] for joint in free_joints]
        generator = randomize(list(product(*free_values)))
    if max_attempts < INF:
        generator = islice(generator, max_attempts)
    max_distances = max_distance*np.ones(len(ik_joints))
    for free_positions in generator:
        for conf in randomize(compute_inverse_kinematics(ikfast.get_ik, base_from_ee, free_positions)):
            difference = difference_fn(current_conf, conf)
            #print(np.round(difference, 3))
            if not violates_limits(robot, ik_joints, conf) and np.less_equal(np.abs(difference), max_distances).all():
                #set_joint_positions(robot, ik_joints, conf)
                yield conf

def closest_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target, **kwargs):
    ik_joints = get_ik_joints(robot, ikfast_info, tool_link)
    current_conf = get_joint_positions(robot, ik_joints)
    solutions = list(ikfast_inverse_kinematics(robot, ikfast_info, tool_link, world_from_target, max_attempts=INF, **kwargs))
    if not solutions:
        return None
    difference_fn = get_difference_fn(robot, ik_joints)
    distance_fn = get_distance_fn(robot, ik_joints)
    closest_conf = min(solutions, key=lambda q: distance_fn(q, current_conf))
    #print(distance_fn(current_conf, closest_conf), np.round(difference_fn(current_conf, closest_conf), 3))
    #set_joint_positions(robot, ik_joints, closest_conf)
    return closest_conf
