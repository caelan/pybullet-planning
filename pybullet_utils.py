import math
import time
from collections import defaultdict, deque
from itertools import product
from motion_planners.rrt_connect import birrt, direct_path
import pybullet as p

import numpy as np

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
REVOLUTE_LIMITS = -np.pi, np.pi
BASE_LIMITS = ([-2.5, -2.5, 0], [2.5, 2.5, 0])

#####################################

class Pose(object):
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation


class Conf(object):
    def __init__(self):
        pass

#####################################

# PR2

def rightarm_from_leftarm(config):
  right_from_left = np.array([-1, 1, -1, 1, -1, 1, 1])
  return config*right_from_left

REST_RIGHT_ARM = rightarm_from_leftarm(REST_LEFT_ARM)
TOP_HOLDING_RIGHT_ARM = rightarm_from_leftarm(TOP_HOLDING_LEFT_ARM)


#####################################

# Simulation

def is_connected():
    return p.getConnectionInfo()['isConnected']

def get_connection():
    return p.getConnectionInfo()['connectionMethod']

def has_gui():
    return get_connection() == p.GUI

#####################################

# Geometry

def invert((point, quat)):
    return p.invertTransform(point, quat)


def multiply((point1, quat1), (point2, quat2)):
    return p.multiplyTransforms(point1, quat1, point2, quat2) # TODO: variable number of args

def unit_from_theta(theta):
    return np.array([np.cos(theta), np.sin(theta)])

def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler)


def euler_from_quat(quat):
    return p.getEulerFromQuaternion(quat)


def z_rotation(theta):
    return quat_from_euler([0, 0, theta])


def matrix_from_quat(quat):
    return p.getMatrixFromQuaternion(quat)


def quat_from_matrix(mat):
    matrix = np.eye(4)
    matrix[:3,:3] = mat
    q = np.empty((4,), dtype=np.float64)
    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    t = np.trace(M)
    if t > M[3, 3]:
        q[3] = t
        q[2] = M[1, 0] - M[0, 1]
        q[1] = M[0, 2] - M[2, 0]
        q[0] = M[2, 1] - M[1, 2]
    else:
        i, j, k = 0, 1, 2
        if M[1, 1] > M[0, 0]:
            i, j, k = 1, 2, 0
        if M[2, 2] > M[i, i]:
            i, j, k = 2, 0, 1
        t = M[i, i] - (M[j, j] + M[k, k]) + M[3, 3]
        q[i] = t
        q[j] = M[i, j] + M[j, i]
        q[k] = M[k, i] + M[i, k]
        q[3] = M[k, j] - M[j, k]
    q *= 0.5 / math.sqrt(t * M[3, 3])
    return q # [x, y, z, w].


def point_from_tform(tform):
    return np.array(tform)[:3,3]


def matrix_from_tform(tform):
    return np.array(tform)[:3,:3]


def tform_from_pose((point, quat)):
    tform = np.eye(4)
    tform[:3,3] = point
    tform[:3,:3] = matrix_from_quat(quat)
    return tform


def pose_from_tform(tform):
    return point_from_tform(tform), quat_from_matrix(matrix_from_tform(tform))

def wrap_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi

def circular_difference(theta2, theta1):
    return wrap_angle(theta2 - theta1)

#####################################

# Joints

def get_num_joints(body):
    return p.getNumJoints(body)


def get_joints(body):
    return range(get_num_joints(body))

def get_joint(body, joint_or_name):
    if type(joint_or_name) is str:
        return joint_from_name(body, joint_or_name)
    return joint_or_name

def get_joint_name(body, joint):
    return p.getJointInfo(body, joint)[1]

def get_joint_names(body):
    return [get_joint_name(body, joint) for joint in get_joints(body)]

def joint_from_name(body, name):
    for joint in xrange(get_num_joints(body)):
        if get_joint_name(body, joint) == name:
            return joint
    raise ValueError(body, name)


def get_joint_position(body, joint):
    return p.getJointState(body, joint)[0]

def get_joint_positions(body, joints):
    return tuple(get_joint_position(body, joint) for joint in joints)

def set_joint_position(body, joint, value):
    p.resetJointState(body, joint, value)

def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        set_joint_position(body, joint, value)


def get_joint_type(body, joint):
    return p.getJointInfo(body, joint)[2]


def is_movable(body, joint):
    return get_joint_type(body, joint) != p.JOINT_FIXED

def get_movable_joints(body): # 45 / 87 on pr2
    return [joint for joint in get_joints(body) if is_movable(body, joint)]


def joint_from_movable(body, index):
    return get_joints(body)[index]


def is_circular(body, joint):
    lower = p.getJointInfo(body, joint)[8]
    upper = p.getJointInfo(body, joint)[9]
    return upper < lower


def get_joint_limits(body, joint):
    if is_circular(body, joint):
        return REVOLUTE_LIMITS
    return p.getJointInfo(body, joint)[8:10]


def get_min_limit(body, joint):
    return get_joint_limits(body, joint)[0]

def get_max_limit(body, joint):
    return get_joint_limits(body, joint)[1]

def get_max_velocity(body, joint):
    return p.getJointInfo(body, joint)[11]

def violates_limit(body, joint, value):
    if not is_circular(body, joint):
        lower, upper = get_joint_limits(body, joint)
        if (value < lower) or (upper < value):
            return True
    return False

def violates_limits(body, joints, values):
    return any(violates_limit(body, joint, value) for joint, value in zip(joints, values))

def wrap_joint(body, joint, value):
    if is_circular(body, joint):
        return wrap_angle(value)
    return value

#####################################

# Links

get_links = get_joints

def get_link_name(body, link):
    return p.getJointInfo(body, link)[12]

def get_link_parent(body, joint):
    return p.getJointInfo(body, joint)[16]

def link_from_name(body, name):
    for link in xrange(get_num_joints(body)):
        if get_link_name(body, link) == name:
            return link
    raise ValueError(body, name)

def get_link_pose(body, link): # Local vs world?
    #point, quat = p.getLinkState(body, link)[0:2] # Local
    #point, quat = p.getLinkState(body, link)[2:4] # World
    point, quat = p.getLinkState(body, link)[4:6] # World
    return point, quat
    #return np.concatenate([point, quat])

def get_adjacent_links(body):
    adjacent = set()
    for link in get_links(body):
        parent = get_link_parent(body, link)
        adjacent.add((link, parent))
        #adjacent.add((parent, link))
    return adjacent


def get_adjacent_fixed_links(body):
    return filter(lambda (link, _): not is_movable(body, link), get_adjacent_links(body))


def get_fixed_links(body):
    edges = defaultdict(list)
    for link, parent in get_adjacent_fixed_links(body):
        edges[link].append(parent)
        edges[parent].append(link)
    visited = set()
    fixed = set()
    for initial_link in get_links(body):
        if initial_link in visited:
            continue
        cluster = [initial_link]
        queue = deque([initial_link])
        visited.add(initial_link)
        while queue:
            for next_link in edges[queue.popleft()]:
                if next_link not in visited:
                    cluster.append(next_link)
                    queue.append(next_link)
                    visited.add(next_link)
        #print cluster
        fixed.update(product(cluster, cluster))
    return fixed

#####################################

# Bodies

def get_num_bodies():
    return p.getNumBodies()


def get_bodies():
    return range(get_num_bodies())

def get_name(body):
    return p.getBodyInfo(body)[1]

def get_body_names():
    return map(get_name, get_bodies())



def body_from_name(name):
    for body in xrange(get_num_bodies()):
        if get_name(body) == name:
            return body
    raise ValueError(name)



def get_base_link(body):
    return p.getBodyInfo(body)[0]

def get_point(body):
    return p.getBasePositionAndOrientation(body)[0]


def get_quat(body):
    return p.getBasePositionAndOrientation(body)[1]

def get_pose(body):
    return p.getBasePositionAndOrientation(body)
    #point, quat = p.getBasePositionAndOrientation(body) # [x,y,z,w]
    #return np.concatenate([point, quat])

def get_base_values(body):
    x, y, _ = get_point(body)
    roll, pitch, yaw = euler_from_quat(get_quat(body))
    assert (abs(roll) < 1e-3) and (abs(pitch) < 1e-3)
    return (x, y, yaw)


def set_base_values(body, values):
    _, _, z = get_point(body)
    x, y, theta = values
    set_point(body, (x, y, z))
    set_quat(body, z_rotation(theta))

def set_point(body, point):
    _, quat = p.getBasePositionAndOrientation(body)
    p.resetBasePositionAndOrientation(body, point, quat)


def set_quat(body, quat):
    p.resetBasePositionAndOrientation(body, get_point(body), quat)

def set_pose(body, point, quat):
    p.resetBasePositionAndOrientation(body, point, quat)

#####################################

# Shapes

def create_box(w, l, h, color=(1, 0, 0, 1)):
    half_extents = [w/2., l/2., h/2.]
    collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    if (color is None) or not has_gui():
        visual_id = -1
    else:
        visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
    return p.createMultiBody(baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id) # basePosition | baseOrientation
    # linkCollisionShapeIndices | linkVisualShapeIndices

def create_mesh():
    raise NotImplementedError()


def create_cylinder(radius, height):
    collision_id =  p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)


def create_capsule(radius, height):
    collision_id = p.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=height)


def create_sphere(radius):
    collision_id = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)


def create_plane():
    collision_id = p.createVisualShape(p.GEOM_PLANE, normal=[])

def get_lower_upper(body):
    return p.getAABB(body)

def get_center_extent(body):
    lower, upper = get_lower_upper(body)
    center = (np.array(lower) + upper) / 2
    extents = (np.array(upper) - lower)
    return center, extents

def get_shape_data(body):
    return p.getVisualShapeData(body)

#####################################

# Collision


def contact_collision():
    p.stepSimulation()
    return len(p.getContactPoints()) != 0


def pairwise_collision(body1, body2, max_distance=0.001): # 10000
    return len(p.getClosestPoints(body1, body2, max_distance)) != 0 # getContactPoints


def pairwise_link_collision(body1, link1, body2, link2, max_distance=0.001): # 10000
    return len(p.getClosestPoints(body1, body2, link1, link2, max_distance)) != 0 # getContactPoints


def get_contact_links(contact):
    _, body1, body2, link1, link2 = contact[:5]
    distance = contact[8]
    return (body1, link1), (body2, link2), distance


def get_colliding_links(body, max_distance=0):
    contacts = p.getClosestPoints(body, body, max_distance) # -1 is the base
    colliding = set()
    for (_, link1), (_, link2), _ in map(get_contact_links, contacts):
        colliding.update([(link1, link2), (link2, link1)])
    return colliding


def self_collision(body, max_distance=0):
    # GetNonAdjacentLinks | GetAdjacentLinks
    contacts = p.getClosestPoints(body, body, max_distance) # -1 is the base
    #print contacts
    adjacent = (get_adjacent_links(body) | get_fixed_links(body))
    #print fixed
    #print sorted(get_adjacent_links(body))
    colliding_not_adjacent = {(link1, link2, distance) for (_, link1), (_, link2), distance in map(get_contact_links, contacts)
           if (link1 != link2) and ((link1, link2) not in adjacent) and ((link2, link1) not in adjacent)}
    colliding_not_adjacent = list(colliding_not_adjacent)
    #print colliding_not_adjacent
    #print [(get_link_name(body, link1), get_link_name(body, link2), distance)
    #       for (link1, link2, distance) in colliding_not_adjacent]
    # TODO: could compute initially colliding links and discount those collisions

    return len(colliding_not_adjacent) != 0


def env_collision(body1):
    for body2 in get_bodies():
        if (body1 != body2) and pairwise_collision(body1, body2):
        #if pairwise_collision(body1, body2):
            return True
    return False


def ray_collision():
    # rayTestBatch
    # rayTest
    raise NotImplementedError()

#####################################

# Motion

def sample_joints(body, joints):
    values = []
    for joint in joints:
        limits = REVOLUTE_LIMITS if is_circular(body, joint) \
            else get_joint_limits(body, joint)
        values.append(np.random.uniform(*limits))
    return tuple(values)



def plan_base_motion(body, end_conf, **kwargs):
    def sample_fn():
        x, y, _ = np.random.uniform(*BASE_LIMITS)
        theta = np.random.uniform(*REVOLUTE_LIMITS)
        return (x, y, theta)

    def difference_fn(q2, q1):
        #return np.array(q2) - np.array(q1)
        dx, dy = np.array(q2[:2]) - np.array(q1[:2])
        dtheta = circular_difference(q2[2], q1[2])
        return (dx, dy, dtheta)

    weights = 1*np.ones(3)
    def distance_fn(q1, q2):
        difference = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, difference * difference))
        #return np.linalg.norm(np.array(q2) - np.array(q1))

    resolutions = 0.05*np.ones(3)
    def extend_fn(q1, q2):
        steps = np.abs(np.divide(difference_fn(q2, q1), resolutions))
        n = int(np.max(steps)) + 1
        q = q1
        for i in xrange(n):
            q = tuple((1. / (n - i)) * np.array(difference_fn(q2, q)) + q)
            yield q
            # TODO: should wrap these joints

    def collision_fn(q):
        set_base_values(body, q)
        return env_collision(body)

    start_conf = get_base_values(body)
    return birrt(start_conf, end_conf, distance_fn,
                 sample_fn, extend_fn, collision_fn, **kwargs)

def plan_joint_motion(body, joints, end_conf, **kwargs):
    assert len(joints) == len(end_conf)

    sample_fn = lambda: sample_joints(body, joints)

    def difference_fn(q2, q1):
        difference = []
        for joint, value2, value1 in zip(joints, q2, q1):
            difference.append((value2 - value1) if is_circular(body, joint)
                              else circular_difference(value2, value1))
        return tuple(difference)

    # TODO: custom weights and step sizes
    weights = 1*np.ones(len(joints))
    def distance_fn(q1, q2):
        diff = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, diff * diff))

    resolutions = 0.05*np.ones(len(joints))
    def extend_fn(q1, q2):
        steps = np.abs(np.divide(difference_fn(q2, q1), resolutions))
        num_steps = int(np.max(steps)) + 1
        q = q1
        for i in xrange(num_steps):
            q = tuple((1. / (num_steps - i)) * np.array(difference_fn(q2, q)) + q)
            yield q
            # TODO: should wrap these joints

    def collision_fn(q):
        if violates_limits(body, joints, q):
            return True
        set_joint_positions(body, joints, q)
        return env_collision(body)

    start_conf = get_joint_positions(body, joints)
    return birrt(start_conf, end_conf, distance_fn,
                 sample_fn, extend_fn, collision_fn, **kwargs)

#####################################

# Placements

def sample_placement(top_body, bottom_body, max_attempts=50):
    bottom_aabb = get_lower_upper(bottom_body)
    for _ in xrange(max_attempts):
        theta = np.random.uniform(*REVOLUTE_LIMITS)
        quat = z_rotation(theta)
        set_quat(top_body, quat)
        center, extent = get_center_extent(top_body)
        lower = (np.array(bottom_aabb[0]) + extent/2)[:2]
        upper = (np.array(bottom_aabb[1]) - extent/2)[:2]
        if np.any(upper < lower):
          continue
        x, y = np.random.uniform(lower, upper)
        z = (bottom_aabb[1] + extent/2.)[2]
        point = np.array([x, y, z]) + (get_point(top_body) - center)
        set_point(top_body, point)
        return point, quat
    return None

#####################################

# Kinematics

def sample_reachable_base(robot, point, max_attempts=50):
    reachable_range = (0.25, 1.0)
    for _ in xrange(max_attempts):
        radius = np.random.uniform(*reachable_range)
        x, y = radius*unit_from_theta(np.random.uniform(-np.pi, np.pi)) + point[:2]
        yaw = np.random.uniform(*REVOLUTE_LIMITS)
        base_values = (x, y, yaw)
        set_base_values(robot, base_values)
        return base_values
        #_, _, z = get_point(robot)
        #point = (x, y, z)
        #set_point(robot, point)
        #quat = z_rotation(yaw)
        #set_quat(robot, quat)
        #return (point, quat)
    return None


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

# Constraints

def remove_constraint(constraint):
    p.removeConstraint(constraint)


def fixed_constraint(body1, link1, body2, link2):
    return p.createConstraint(body1, link1, body2, link2,
                              p.JOINT_FIXED, jointAxis=[0]*3, # JOINT_FIXED
                              parentFramePosition=[0]*3,
                              childFramePosition=[0]*3)

#####################################

# Control

def set_joint(body, joint, value):
    p.setJointMotorControl2(bodyUniqueId=body,
                            jointIndex=joint,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=value,
                            force=maxForce)


def control_joints(body, joints, positions):
    # TODO: the whole PR2 seems to jitter
    kp = 1.0
    kv = 0.3
    p.setJointMotorControlArray(body, joints, p.POSITION_CONTROL, targetPositions=positions,
                                targetVelocities=[0.0] * len(joints)) #,
                                #positionGains=[kp] * len(joints),
                                #velocityGains=[kv] * len(joints),)
                                #forces=[])