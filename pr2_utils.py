import numpy as np
import math
import pybullet as p
import time

#####################################

# PR2
from pybullet_utils import get_joint_limits, multiply, get_max_velocity, get_movable_joints, \
    get_link_pose, joint_from_name, link_from_name, set_joint_position, set_joint_positions, get_joint_positions, \
    get_min_limit, get_max_limit, quat_from_euler, get_joints

TOP_HOLDING_LEFT_ARM = [0.67717021, -0.34313199, 1.2, -1.46688405, 1.24223229, -1.95442826, 2.22254125]
SIDE_HOLDING_LEFT_ARM = [0.39277395, 0.33330058, 0., -1.52238431, 2.72170996, -1.21946936, -2.98914779]
REST_LEFT_ARM = [2.13539289, 1.29629967, 3.74999698, -0.15000005, 10000., -0.10000004, 10000.]
WIDE_LEFT_ARM = [1.5806603449288885, -0.14239066980481405, 1.4484623937179126, -1.4851759349218694, 1.3911839347271555, -1.6531320011389408, -2.978586584568441]
CENTER_LEFT_ARM = [-0.07133691252641006, -0.052973836083405494, 1.5741805775919033, -1.4481146328076862, 1.571782540186805, -1.4891468812835686, -9.413338322697955]
WIDE_RIGHT_ARM = [-1.3175723551150083, -0.09536552225976803, -1.396727055561703, -1.4433371993320296, -1.5334243909312468, -1.7298129320065025, 6.230244924007009]
LEFT_ARM_LINK = 'l_gripper_palm_link' # l_gripper_palm_link | l_gripper_tool_frame
LEFT_JOINT_NAMES = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
                    'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
RIGHT_JOINT_NAMES = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                     'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
HEAD_JOINT_NAMES = ['head_pan_joint', 'head_tilt_joint']
LEFT_GRIPPER_NAME = 'l_gripper_l_finger_joint'
LEFT_TOOL_NAME = 'l_gripper_tool_frame' # l_gripper_tool_joint | l_gripper_tool_frame
LEFT_GRIPPER = 'l_gripper_l_finger_joint' # l_gripper_l_finger_joint | l_gripper_joint
RIGHT_GRIPPER = 'r_gripper_l_finger_joint' # r_gripper_l_finger_joint | r_gripper_joint

TOOL_TFORM = [[0., 0., 1., 0.18],
              [0., 1., 0., 0.],
              [-1., 0., 0., 0.],
              [0., 0., 0., 1.]]
TOOL_POSE = ([0.18, 0., 0.],
             [0., 0.70710678, 0., 0.70710678])
TOOL_DIRECTION = [ 0., 0., 1.]
TORSO_JOINT = 'torso_lift_joint'


def rightarm_from_leftarm(config):
  right_from_left = np.array([-1, 1, -1, 1, -1, 1, 1])
  return config*right_from_left


REST_RIGHT_ARM = rightarm_from_leftarm(REST_LEFT_ARM)
TOP_HOLDING_RIGHT_ARM = rightarm_from_leftarm(TOP_HOLDING_LEFT_ARM)

#####################################

# End-effectors

def get_gripper_pose(robot):
    # world_from_gripper * gripper_from_tool * tool_from_object = world_from_object
    pose = multiply(get_link_pose(robot, link_from_name(robot, LEFT_ARM_LINK)), TOOL_POSE)
    #pose = get_link_pose(robot, link_from_name(robot, LEFT_TOOL_NAME))
    return pose

def close_gripper(robot, joint):
    set_joint_position(robot, joint, get_min_limit(robot, joint))


def open_gripper(robot, joint):
    set_joint_position(robot, joint, get_max_limit(robot, joint))


GRASP_LENGTH = 0.04
MAX_GRASP_WIDTH = 0.07

def get_top_grasps(body, under=False, limits=True, grasp_length=GRASP_LENGTH):
    # w, l, h = np.max(mesh.vertices, axis=0) - \
    #          np.min(mesh.vertices, axis=0)
    h = 0.15
    reflect_z = (np.zeros(3), quat_from_euler([0, math.pi, 0]))
    translate = ([0, 0, h / 2 - grasp_length], quat_from_euler(np.zeros(3)))
    # if not limits or (w <= MAX_GRASP_WIDTH):
    for i in range(1 + under):
        rotate_z = (np.zeros(3), quat_from_euler([0, 0, math.pi / 2 + i * math.pi]))
        yield multiply(multiply(translate, rotate_z), reflect_z)
    # if not limits or (l <= MAX_GRASP_WIDTH):
    for i in range(1 + under):
        rotate_z = (np.zeros(3), quat_from_euler([0, 0, i * math.pi]))
        yield multiply(multiply(translate, rotate_z), reflect_z)


#####################################

def inverse_kinematics(robot, target_pose, flag=True):
    #print get_base_link(robot)
    #base_pose = get_link_pose(robot, link_from_name(robot, get_base_link(robot)))
    #pose = multiply(invert(get_pose(robot)), pose)
    (point, quat) = target_pose
    link = link_from_name(robot, LEFT_ARM_LINK)

    movable_joints = get_movable_joints(robot)
    joints = get_joints(robot)
    current_conf = get_joint_positions(robot, joints)
    # TODO: constrain not moving
    left_joints = [joint_from_name(robot, name) for name in LEFT_JOINT_NAMES]
    # [joint_from_name(robot, name) for name in RIGHT_JOINT_NAMES]

    # https://github.com/bulletphysics/bullet3/blob/389d7aaa798e5564028ce75091a3eac6a5f76ea8/examples/SharedMemory/PhysicsClientC_API.cpp
    # Need to have the correct number

    # TODO: sample other values for teh arm joints as the reference conf

    min_limits = [get_joint_limits(robot, joint)[0] for joint in joints]
    max_limits = [get_joint_limits(robot, joint)[1] for joint in joints]
    min_limits = [get_joint_limits(robot, joint)[0] if joint in left_joints else current_conf[i]
                  for i, joint in enumerate(joints)]
    max_limits = [get_joint_limits(robot, joint)[1] if joint in left_joints else current_conf[i]
                  for i, joint in enumerate(joints)]
    #min_limits = current_conf
    #max_limits = current_conf
    max_velocities = [get_max_velocity(robot, joint) for joint in joints] # Range of Jacobian
    max_velocities = [10000]*len(joints) # TODO: cannot have zero velocities
    # TODO: larger definitely better for velocities
    print min_limits
    print max_limits
    print max_velocities
    print
    #print max_velocities
    #kinematic_conf = p.calculateInverseKinematics(robot, link, point, lowerLimits=min_limits,
    #                                    upperLimits=max_limits, jointRanges=max_velocities,
    #                                    restPoses=current_conf)
    #kinematic_conf = p.calculateInverseKinematics(robot, link, point, quat, lowerLimits=min_limits,
    #                                    upperLimits=max_limits, jointRanges=max_velocities,
    #                                    restPoses=current_conf)

    # https://github.com/bulletphysics/bullet3/blob/c1ba04a5809f7831fa2dee684d6747951a5da602/examples/pybullet/examples/inverse_kinematics_husky_kuka.py

    #damping = tuple(0.1*np.ones(len(movable_joints)))
    damping = tuple(0.1*np.ones(len(joints)))
    #print damping
    #min_limits = 'fis'
    #max_limits = 'stuff' # TODO: these are not used?
    #max_velocities = 'what'
    #current_conf = 'aefawfe'

    # TODO: fix particular joints?
    t0 = time.time()
    max_iterations = 100
    kinematic_conf = get_joint_positions(robot, movable_joints)
    for iterations in xrange(max_iterations): # 0.000863273143768 / iteration
        #print kinematic_conf
        #kinematic_conf = p.calculateInverseKinematics(robot, link, point, quat)
        #kinematic_conf = p.calculateInverseKinematics(robot, link, point)
        if flag:
            kinematic_conf = p.calculateInverseKinematics(robot, link, point,
                                                          quat,
                                                          lowerLimits=min_limits, upperLimits=max_limits,
                                                          jointRanges=max_velocities, restPoses=current_conf,
                                                          #jointDamping=damping,
                                                  )
        else:
            kinematic_conf = p.calculateInverseKinematics(robot, link, point, quat)
        #kinematic_conf = p.calculateInverseKinematics(robot, link, point, quat,
        #                                              min_limits, max_limits,
        #                                              max_velocities, current_conf)
        if (kinematic_conf is None) or any(map(math.isnan, kinematic_conf)):
            break
        set_joint_positions(robot, movable_joints, kinematic_conf)
        link_point, link_quat = get_link_pose(robot, link)
        #print link_point, link_quat
        if np.allclose(link_point, point, atol=1e-3) and np.allclose(link_quat, quat, atol=1e-3):
            print iterations
            break
        #print link_point, link_quat
    else:
        #return None
        return kinematic_conf
    total_time = (time.time() - t0)
    print total_time
    print (time.time() - t0)/max_iterations
    #kinematic_conf = p.calculateInverseKinematics(robot, link, point)
    return kinematic_conf