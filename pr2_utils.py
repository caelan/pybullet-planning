import os
import random

import numpy as np
import math
import pybullet as p
import time

#####################################

# PR2
from pybullet_utils import get_joint_limits, multiply, get_max_velocity, get_movable_joints, \
    get_link_pose, joint_from_name, link_from_name, set_joint_position, set_joint_positions, get_joint_positions, \
    get_min_limit, get_max_limit, quat_from_euler, get_joints, violates_limits, read_pickle, set_pose, point_from_pose, \
    sample_reachable_base, set_base_values, get_pose, sample_placement, invert, pairwise_collision, get_body_name, \
    euler_from_quat, unit_point, unit_quat, unit_pose, get_center_extent, write_pickle

TOP_HOLDING_LEFT_ARM = [0.67717021, -0.34313199, 1.2, -1.46688405, 1.24223229, -1.95442826, 2.22254125]
SIDE_HOLDING_LEFT_ARM = [0.39277395, 0.33330058, 0., -1.52238431, 2.72170996, -1.21946936, -2.98914779]
REST_LEFT_ARM = [2.13539289, 1.29629967, 3.74999698, -0.15000005, 10000., -0.10000004, 10000.]
WIDE_LEFT_ARM = [1.5806603449288885, -0.14239066980481405, 1.4484623937179126, -1.4851759349218694, 1.3911839347271555, -1.6531320011389408, -2.978586584568441]
CENTER_LEFT_ARM = [-0.07133691252641006, -0.052973836083405494, 1.5741805775919033, -1.4481146328076862, 1.571782540186805, -1.4891468812835686, -9.413338322697955]
WIDE_RIGHT_ARM = [-1.3175723551150083, -0.09536552225976803, -1.396727055561703, -1.4433371993320296, -1.5334243909312468, -1.7298129320065025, 6.230244924007009]

LEFT_JOINT_NAMES = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
                    'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
RIGHT_JOINT_NAMES = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                     'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
ARM_JOINT_NAMES = {
    'left': LEFT_JOINT_NAMES,
    'right': RIGHT_JOINT_NAMES,
}

LEFT_ARM_LINK = 'l_gripper_palm_link' # l_gripper_palm_link | l_gripper_tool_frame
ARM_LINK_NAMES = {
    'left': LEFT_ARM_LINK,
    'right': 'r_gripper_palm_link',
}

HEAD_JOINT_NAMES = ['head_pan_joint', 'head_tilt_joint']
LEFT_TOOL_NAME = 'l_gripper_tool_frame' # l_gripper_tool_joint | l_gripper_tool_frame

LEFT_GRIPPER = 'l_gripper_l_finger_joint' # l_gripper_l_finger_joint | l_gripper_joint
RIGHT_GRIPPER = 'r_gripper_l_finger_joint' # r_gripper_l_finger_joint | r_gripper_joint

GRIPPER_JOINT_NAMES = {
    'left': ['l_gripper_l_finger_joint', 'l_gripper_r_finger_joint'],
    'right': ['r_gripper_l_finger_joint', 'r_gripper_r_finger_joint'],
}

#TOOL_TFORM = [[0., 0., 1., 0.18],
#              [0., 1., 0., 0.],
#              [-1., 0., 0., 0.],
#              [0., 0., 0., 1.]]
TOOL_POSE = ([0.18, 0., 0.],
             [0., 0.70710678, 0., 0.70710678])

TOOL_DIRECTION = [ 0., 0., 1.]
TORSO_JOINT = 'torso_lift_joint'

def rightarm_from_leftarm(config):
  right_from_left = np.array([-1, 1, -1, 1, -1, 1, 1])
  return config*right_from_left

def arm_conf(arm, config):
  assert arm in ARM_JOINT_NAMES
  if arm == 'left':
      return config
  else:
      return rightarm_from_leftarm(config)


REST_RIGHT_ARM = rightarm_from_leftarm(REST_LEFT_ARM)
TOP_HOLDING_RIGHT_ARM = rightarm_from_leftarm(TOP_HOLDING_LEFT_ARM)

def get_carry_conf(arm, grasp_type):
    if grasp_type == 'top':
        return arm_conf(arm, TOP_HOLDING_LEFT_ARM)
    elif grasp_type == 'side':
        return arm_conf(arm, SIDE_HOLDING_LEFT_ARM)
    else:
        raise NotImplementedError()

def get_other_arm(arm):
    for other_arm in ARM_JOINT_NAMES:
        if other_arm != arm:
            return other_arm
    raise ValueError(arm)

#####################################

# End-effectors

def get_arm_joints(robot, arm):
    assert arm in ARM_JOINT_NAMES
    return tuple(joint_from_name(robot, name) for name in ARM_JOINT_NAMES[arm])

def get_arm_conf(robot, arm):
    return get_joint_positions(robot, get_arm_joints(robot, arm))

def set_arm_conf(robot, arm, conf):
    set_joint_positions(robot, get_arm_joints(robot, arm), conf)

def get_gripper_link(robot, arm):
    assert arm in ARM_LINK_NAMES
    return link_from_name(robot, ARM_LINK_NAMES[arm])

def get_gripper_pose(robot):
    # world_from_gripper * gripper_from_tool * tool_from_object = world_from_object
    pose = multiply(get_link_pose(robot, link_from_name(robot, LEFT_ARM_LINK)), TOOL_POSE)
    #pose = get_link_pose(robot, link_from_name(robot, LEFT_TOOL_NAME))
    return pose

def close_gripper(robot, joint):
    set_joint_position(robot, joint, get_min_limit(robot, joint))

def open_gripper(robot, joint):
    #set_joint_position(robot, joint, get_max_limit(robot, joint)/2.)
    set_joint_position(robot, joint, get_max_limit(robot, joint))

def open_arm(robot, arm): # TODO: these are mirrored on the pr2
    assert arm in GRIPPER_JOINT_NAMES
    for name in GRIPPER_JOINT_NAMES[arm]:
        joint = joint_from_name(robot, name)
        open_gripper(robot, joint)

def close_arm(robot, arm):
    assert arm in GRIPPER_JOINT_NAMES
    for name in GRIPPER_JOINT_NAMES[arm]:
        joint = joint_from_name(robot, name)
        close_gripper(robot, joint)

#GRASP_LENGTH = 0.04
GRASP_LENGTH = 0.
MAX_GRASP_WIDTH = 0.07

def get_top_grasps(body, under=False, limits=True, grasp_length=GRASP_LENGTH):
    pose = get_pose(body)
    set_pose(body, unit_pose())
    center, (w, l, h) = get_center_extent(body)
    reflect_z = (np.zeros(3), quat_from_euler([0, math.pi, 0]))
    translate = ([0, 0, h / 2 - grasp_length], unit_quat())
    grasps = []
    # if not limits or (w <= MAX_GRASP_WIDTH):
    if True:
        for i in range(1 + under):
            rotate_z = (unit_point(), quat_from_euler([0, 0, math.pi / 2 + i * math.pi]))
            grasps += [multiply(TOOL_POSE, translate, rotate_z, reflect_z)]
    # if not limits or (l <= MAX_GRASP_WIDTH):
    if True:
        for i in range(1 + under):
            rotate_z = (unit_point(), quat_from_euler([0, 0, i * math.pi]))
            grasps += [multiply(TOOL_POSE, translate, rotate_z, reflect_z)]
    set_pose(body, pose)
    return grasps

def get_side_grasps(body, under=False, limits=True, grasp_length=GRASP_LENGTH):
  pose = get_pose(body)
  set_pose(body, unit_pose())
  center, (w, l, h) = get_center_extent(body)
  grasps = []
  for j in range(1 + under):
    swap_xz = (unit_point(), quat_from_euler([0, -math.pi/2 + j*math.pi, 0]))
    #if not limits or (w <= MAX_GRASP_WIDTH):
    if True:
      translate = ([0, 0, l / 2 - grasp_length], unit_quat())
      for i in range(2):
        rotate_z = (unit_point(), quat_from_euler([math.pi / 2 + i * math.pi, 0, 0]))
        grasps += [multiply(TOOL_POSE, translate, rotate_z, swap_xz)] #, np.array([w])
    if True:
    #if not limits or (l <= MAX_GRASP_WIDTH):
      translate = ([0, 0, w / 2 - grasp_length], unit_quat())
      for i in range(2):
        rotate_z = (unit_point(), quat_from_euler([i * math.pi, 0, 0]))
        grasps += [multiply(TOOL_POSE, translate, rotate_z, swap_xz)] #, np.array([l])
  set_pose(body, pose)
  return grasps

def get_x_presses(body, max_orientations=4):
  pose = get_pose(body)
  set_pose(body, unit_pose())
  center, (w, l, h) = get_center_extent(body)
  grasps = []
  for j in xrange(max_orientations):
      swap_xz = (unit_point(), quat_from_euler([0, -math.pi/2 + j*math.pi, 0]))
      translate = ([0, 0, w / 2], unit_quat())
      grasps += [multiply(TOOL_POSE, translate, swap_xz)]
  set_pose(body, pose)
  return grasps

GET_GRASPS = {
    'top': get_top_grasps,
    'side': get_side_grasps,
    'press': get_x_presses,
}

#####################################

def inverse_kinematics_helper(robot, link, (point, quat),
                              null_space=False, max_iterations=200, tolerance=1e-3):
    # https://github.com/bulletphysics/bullet3/blob/389d7aaa798e5564028ce75091a3eac6a5f76ea8/examples/SharedMemory/PhysicsClientC_API.cpp
    # https://github.com/bulletphysics/bullet3/blob/c1ba04a5809f7831fa2dee684d6747951a5da602/examples/pybullet/examples/inverse_kinematics_husky_kuka.py
    joints = get_joints(robot) # Need to have all joints (although only movable returned)
    movable_joints = get_movable_joints(robot)
    current_conf = get_joint_positions(robot, joints)

    # TODO: sample other values for the arm joints as the reference conf
    min_limits = [get_joint_limits(robot, joint)[0] for joint in joints]
    max_limits = [get_joint_limits(robot, joint)[1] for joint in joints]
    #min_limits = current_conf
    #max_limits = current_conf
    #max_velocities = [get_max_velocity(robot, joint) for joint in joints] # Range of Jacobian
    max_velocities = [10000]*len(joints)
    # TODO: cannot have zero velocities
    # TODO: larger definitely better for velocities
    #damping = tuple(0.1*np.ones(len(joints)))

    #t0 = time.time()
    kinematic_conf = get_joint_positions(robot, movable_joints)
    for iterations in xrange(max_iterations): # 0.000863273143768 / iteration
        # TODO: return none if no progress
        if null_space:
            kinematic_conf = p.calculateInverseKinematics(robot, link, point, quat,
                                                          lowerLimits=min_limits, upperLimits=max_limits,
                                                          jointRanges=max_velocities, restPoses=current_conf,
                                                          #jointDamping=damping,
                                                  )
        else:
            kinematic_conf = p.calculateInverseKinematics(robot, link, point, quat)
        if (kinematic_conf is None) or any(map(math.isnan, kinematic_conf)):
            return None
        set_joint_positions(robot, movable_joints, kinematic_conf)
        link_point, link_quat = get_link_pose(robot, link)
        if np.allclose(link_point, point, atol=tolerance) and np.allclose(link_quat, quat, atol=tolerance):
            #print iterations
            break
    else:
        return None
    if violates_limits(robot, movable_joints, kinematic_conf):
        return None
    #total_time = (time.time() - t0)
    #print total_time
    #print (time.time() - t0)/max_iterations
    return kinematic_conf

def inverse_kinematics(robot, target_pose):
    #left_joints = [joint_from_name(robot, name) for name in LEFT_JOINT_NAMES]
    #min_limits = [get_joint_limits(robot, joint)[0] if joint in left_joints else current_conf[i]
    #              for i, joint in enumerate(joints)]
    #max_limits = [get_joint_limits(robot, joint)[1] if joint in left_joints else current_conf[i]
    #              for i, joint in enumerate(joints)]
    link = link_from_name(robot, LEFT_ARM_LINK)
    return inverse_kinematics_helper(robot, link, target_pose)

#####################################

DATABASES_DIR = 'databases'
IR_FILENAME = '{}_{}_ir.pickle'


def load_inverse_reachability(arm, grasp_type):
    filename = IR_FILENAME.format(grasp_type, arm)
    path = os.path.join(DATABASES_DIR, filename)
    return read_pickle(path)['gripper_from_base']


def learned_pose_generator(robot, gripper_pose, arm, grasp_type):
    gripper_from_base_list = load_inverse_reachability(arm, grasp_type)
    random.shuffle(gripper_from_base_list)
    for gripper_from_base in gripper_from_base_list:
        base_point, base_quat = multiply(gripper_pose, gripper_from_base)
        x, y, _ = base_point
        _, _, theta = euler_from_quat(base_quat)
        base_values = (x, y, theta)
        set_base_values(robot, base_values)
        yield get_pose(robot)


def uniform_pose_generator(robot, gripper_pose, **kwargs):
    point = point_from_pose(gripper_pose)
    while True:
        base_values = sample_reachable_base(robot, point)
        set_base_values(robot, base_values)
        yield get_pose(robot)


def create_inverse_reachability(robot, body, table, arm, grasp_type, num_samples=500):
    link = get_gripper_link(robot, arm)
    movable_joints = get_movable_joints(robot)
    default_conf = get_joint_positions(robot, movable_joints)
    gripper_from_base_list = []
    grasps = GET_GRASPS[grasp_type](body)

    while len(gripper_from_base_list) < num_samples:
        box_pose = sample_placement(body, table)
        set_pose(body, box_pose)
        grasp_pose = random.choice(grasps)
        gripper_pose = multiply(box_pose, invert(grasp_pose))
        set_joint_positions(robot, movable_joints, default_conf)
        set_pose(robot, next(uniform_pose_generator(robot, gripper_pose)))
        if pairwise_collision(robot, table):
            continue
        conf = inverse_kinematics_helper(robot, link, gripper_pose)
        if (conf is None) or pairwise_collision(robot, table):
            continue
        gripper_from_base = multiply(invert(get_link_pose(robot, link)), get_pose(robot))
        gripper_from_base_list.append(gripper_from_base)
        print '{} / {}'.format(len(gripper_from_base_list), num_samples)

    filename = IR_FILENAME.format(grasp_type, arm)
    path = os.path.join(DATABASES_DIR, filename)
    data = {
        'filename': filename,
        'robot': get_body_name(robot),
        'grasp_type': grasp_type,
        'arg': arm,
        'carry_conf': get_carry_conf(arm, grasp_type),
        'gripper_link': link,
        'gripper_from_base': gripper_from_base_list,
    }
    write_pickle(path, data)
    return path