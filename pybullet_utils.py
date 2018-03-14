import math
import time
from collections import defaultdict, deque
from itertools import product

import numpy as np
import pybullet as p
import pybullet_data
import pickle
from motion_planners.rrt_connect import birrt

BASE_LIMITS = ([-2.5, -2.5, 0], [2.5, 2.5, 0])
REVOLUTE_LIMITS = -np.pi, np.pi

#####################################

def write_pickle(filename, data):  # NOTE - cannot pickle lambda or nested functions
    with open(filename, 'wb') as f:
        pickle.dump(data, f)

def read_pickle(filename):
    with open(filename, 'rb') as f:
        return pickle.load(f)

#####################################

# Simulation

def connect(use_gui=True, shadows=True):
    sim_id = p.connect(p.GUI) if use_gui else p.connect(p.DIRECT)
    if use_gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, shadows)
    #p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)
    #p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    #p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    #p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, 1)
    #p.configureDebugVisualizer(p.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
    #p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
    #p.configureDebugVisualizer(p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)
    #p.configureDebugVisualizer(p.COV_ENABLE_VR_RENDER_CONTROLLERS, 0)
    #p.configureDebugVisualizer(p.COV_ENABLE_VR_PICKING, 0)
    #p.configureDebugVisualizer(p.COV_ENABLE_VR_TELEPORTING, 0)
    return sim_id

def disconnect():
    return p.disconnect()

def is_connected():
    return p.getConnectionInfo()['isConnected']

def get_connection():
    return p.getConnectionInfo()['connectionMethod']

def has_gui():
    return get_connection() == p.GUI

def add_data_path():
    return p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally

def enable_gravity():
    p.setGravity(0, 0, -10)

def disable_gravity():
    p.setGravity(0, 0, 0)

def step_simulation():
    p.stepSimulation()

def update_state():
    for body in get_bodies():
        get_pose(body)
        for joint in get_joints(body):
            get_joint_position(body, joint)
    #p.getKeyboardEvents()
    #p.getMouseEvents()

def reset_simulation():
    p.resetSimulation()

#####################################

# Geometry

def invert((point, quat)):
    return p.invertTransform(point, quat)

#def multiply((point1, quat1), (point2, quat2)):
#    return p.multiplyTransforms(point1, quat1, point2, quat2)

def multiply(*poses):
    pose = poses[0]
    for next_pose in poses[1:]:
        pose = p.multiplyTransforms(pose[0], pose[1], *next_pose)
    return pose

def unit_from_theta(theta):
    return np.array([np.cos(theta), np.sin(theta)])

def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler)


def euler_from_quat(quat):
    return p.getEulerFromQuaternion(quat)

def unit_point():
    return (0., 0., 0.)

def unit_quat():
    return quat_from_euler([0, 0, 0])

def unit_pose():
    return (unit_point(), unit_quat())

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

def point_from_pose(pose):
    return pose[0]

def quat_from_pose(pose):
    return pose[1]

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

def get_body_name(body):
    return p.getBodyInfo(body)[1]

def get_body_names():
    return map(get_body_name, get_bodies())



def body_from_name(name):
    for body in xrange(get_num_bodies()):
        if get_body_name(body) == name:
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

#def set_pose(body, point, quat):
def set_pose(body, (point, quat)):
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

def get_shape_data(body):
    return p.getVisualShapeData(body)

#####################################

def get_lower_upper(body):
    return p.getAABB(body)

def get_center_extent(body):
    lower, upper = get_lower_upper(body)
    center = (np.array(lower) + upper) / 2
    extents = (np.array(upper) - lower)
    return center, extents

def aabb2d_from_aabb((lower, upper)):
    return lower[:2], upper[:2]

def aabb_contains(contained, container):
    lower1, upper1 = contained
    lower2, upper2 = container
    return np.all(lower2 <= lower1) and np.all(upper1 <= upper2)

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


def get_colliding_links(body, max_distance=-0.001):
    contacts = p.getClosestPoints(body, body, max_distance) # -1 is the base
    colliding = set()
    for (_, link1), (_, link2), _ in map(get_contact_links, contacts):
        colliding.update([(link1, link2), (link2, link1)])
    return colliding

def get_safe_colliding_links(body):
    return get_adjacent_links(body) | get_fixed_links(body)

def self_collision(body, max_distance=0):
    # GetNonAdjacentLinks | GetAdjacentLinks
    contacts = p.getClosestPoints(body, body, max_distance) # -1 is the base
    #print contacts
    adjacent = get_safe_colliding_links(body)
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

def filtered_self_collision(body, acceptable=tuple(), max_distance=0):
    contacts = p.getClosestPoints(body, body, max_distance) # -1 is the base
    for (_, link1), (_, link2), _ in map(get_contact_links, contacts):
        if (link1 != link2) and (link1, link2) not in acceptable:
            return True
    return False

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

def supports_body(top_body, bottom_body, epsilon=1e-2): # TODO: above / below
    top_aabb = get_lower_upper(top_body)
    bottom_aabb = get_lower_upper(bottom_body)
    top_z_min = top_aabb[0][2]
    bottom_z_max = bottom_aabb[1][2]
    return (bottom_z_max <= top_z_min <= (bottom_z_max + epsilon)) and \
           (aabb_contains(aabb2d_from_aabb(top_aabb), aabb2d_from_aabb(bottom_aabb)))

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

def control_joint(body, joint, value):
    max_force = 5
    return p.setJointMotorControl2(bodyUniqueId=body,
                            jointIndex=joint,
                            controlMode=p.POSITION_CONTROL,
                            targetPosition=value,
                            targetVelocity=0,
                            force=max_force)


def control_joints(body, joints, positions):
    # TODO: the whole PR2 seems to jitter
    kp = 1.0
    kv = 0.3
    return p.setJointMotorControlArray(body, joints, p.POSITION_CONTROL, targetPositions=positions,
                                targetVelocities=[0.0] * len(joints)) #,
                                #positionGains=[kp] * len(joints),
                                #velocityGains=[kv] * len(joints),)
                                #forces=[])