from __future__ import print_function

import sys
import math
import time
from collections import defaultdict, deque, namedtuple
from itertools import product

# from future_builtins import map, filter
# from builtins import input # TODO - use future
try:
   input = raw_input
except NameError:
   pass

import numpy as np
import pybullet as p
import pickle
from motion_planners.rrt_connect import birrt, direct_path
from transformations import quaternion_from_matrix

# https://stackoverflow.com/questions/21892989/what-is-the-good-python3-equivalent-for-auto-tuple-unpacking-in-lambda

REVOLUTE_LIMITS = -np.pi, np.pi
INF = np.inf

#####################################

# Models

DRAKE_IIWA_URDF = "models/drake/iiwa_description/urdf/iiwa14_polytope_collision.urdf"

KUKA_IIWA_URDF = "kuka_iiwa/model.urdf"
KUKA_IIWA_GRIPPER_SDF = "kuka_iiwa/kuka_with_gripper.sdf"
R2D2_URDF = "r2d2.urdf"
MINITAUR_URDF = "quadruped/minitaur.urdf"
HUMANOID_MJCF = "mjcf/humanoid.xml"
HUSKY_URDF = "husky/husky.urdf"
KIVA_SHELF_SDF = "kiva_shelf/model.sdf"
SMALL_BLOCK_URDF = "models/drake/objects/block_for_pick_and_place.urdf"
BLOCK_URDF = "models/drake/objects/block_for_pick_and_place_mid_size.urdf"
SINK_URDF = 'models/sink.urdf'
STOVE_URDF = 'models/stove.urdf'

#####################################

# I/O

def write_pickle(filename, data):  # NOTE - cannot pickle lambda or nested functions
    with open(filename, 'wb') as f:
        pickle.dump(data, f)

def read_pickle(filename):
    with open(filename, 'rb') as f:
        return pickle.load(f)

#####################################

# Simulation

def load_model(model_file, pose=None, fixed_base=True):
    add_data_path()
    if model_file.endswith('.urdf'):
        body = p.loadURDF(model_file, useFixedBase=fixed_base)
    elif model_file.endswith('.sdf'):
        body = p.loadSDF(model_file)
    elif model_file.endswith('.xml'):
        body = p.loadMJCF(model_file)
    elif model_file.endswith('.bullet'):
        body = p.loadBullet(model_file)
    else:
        raise ValueError(model_file)
    if pose is not None:
        set_pose(body, pose)
    return body

def wait_for_duration(duration, dt=0):
    t0 = time.time()
    while (time.time() - t0) <= duration:
        step_simulation()
        time.sleep(dt)
    # TODO: wait until keypress

def wait_for_interrupt(max_time=np.inf):
    print('Press Ctrl-C to continue')
    try:
        wait_for_duration(max_time)
    except KeyboardInterrupt:
        pass
    finally:
        print()

def wait_for_input(s=''):
    print(s)
    while True:
        step_simulation()
        line = sys.stdin.readline()
        if line:
            print('Fish')
        #events = p.getKeyboardEvents() # TODO: only works when the viewer is in focus
        #if events:
        #    print(events)
        # https://docs.python.org/2/library/select.html


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

def get_data_path():
    import pybullet_data
    return pybullet_data.getDataPath()

def add_data_path():
    return p.setAdditionalSearchPath(get_data_path())

def enable_gravity():
    p.setGravity(0, 0, -10)

def disable_gravity():
    p.setGravity(0, 0, 0)

def step_simulation():
    p.stepSimulation()

def enable_real_time():
    p.setRealTimeSimulation(1)

def disable_real_time():
    p.setRealTimeSimulation(0)

def update_state():
    for body in get_bodies():
        get_pose(body)
        for joint in get_joints(body):
            get_joint_position(body, joint)
    #p.getKeyboardEvents()
    #p.getMouseEvents()

def reset_simulation():
    p.resetSimulation()

def get_camera():
    return p.getDebugVisualizerCamera()

def set_camera(yaw, pitch, distance, target_position):
    p.resetDebugVisualizerCamera(distance, yaw, pitch, target_position)

def set_default_camera():
    set_camera(160, -35, 2.5, Point())

#####################################

# Geometry

def Point(x=0., y=0., z=0.):
    return np.array([x, y, z])

def Euler(roll=0., pitch=0., yaw=0.):
    return np.array([roll, pitch, yaw])

def Pose(point=None, euler=None):
    point = Point() if point is None else point
    euler = Euler() if euler is None else euler
    return (point, quat_from_euler(euler))

def Pose2d(x=0., y=0., yaw=0.):
    return np.array([x, y, yaw])

def invert(pose):
    (point, quat) = pose
    return p.invertTransform(point, quat)

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
    return quaternion_from_matrix(matrix)

def point_from_tform(tform):
    return np.array(tform)[:3,3]

def matrix_from_tform(tform):
    return np.array(tform)[:3,:3]

def point_from_pose(pose):
    return pose[0]

def quat_from_pose(pose):
    return pose[1]

def tform_from_pose(pose):
    (point, quat) = pose
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

def base_values_from_pose(pose):
    (point, quat) = pose
    x, y, _ = point
    roll, pitch, yaw = euler_from_quat(quat)
    assert (abs(roll) < 1e-3) and (abs(pitch) < 1e-3)
    return (x, y, yaw)

def pose_from_base_values(base_values, default_pose):
    x, y, yaw = base_values
    _, _, z = default_pose[0]
    roll, pitch, _ = euler_from_quat(default_pose[1])
    return (x, y, z), quat_from_euler([roll, pitch, yaw])

#####################################

# Bodies

def remove_body(body):
    return p.removeBody(body)

def get_num_bodies():
    return p.getNumBodies()

def get_bodies():
    return list(range(get_num_bodies()))

def get_base_name(body):
    return p.getBodyInfo(body)[0]

def get_body_name(body):
    return p.getBodyInfo(body)[1]

def get_body_names():
    return map(get_body_name, get_bodies())

def body_from_name(name):
    for body in range(get_num_bodies()):
        if get_body_name(body) == name:
            return body
    raise ValueError(name)

def get_point(body):
    return p.getBasePositionAndOrientation(body)[0]

def get_quat(body):
    return p.getBasePositionAndOrientation(body)[1] # [x,y,z,w]

def get_pose(body):
    return p.getBasePositionAndOrientation(body)
    #return np.concatenate([point, quat])

def get_base_values(body):
    return base_values_from_pose(get_pose(body))

def set_point(body, point):
    p.resetBasePositionAndOrientation(body, point, get_quat(body))

def set_quat(body, quat):
    p.resetBasePositionAndOrientation(body, get_point(body), quat)

def set_pose(body, pose):
    (point, quat) = pose
    p.resetBasePositionAndOrientation(body, point, quat)

def set_base_values(body, values):
    _, _, z = get_point(body)
    x, y, theta = values
    set_point(body, (x, y, z))
    set_quat(body, z_rotation(theta))

def dump_world():
    for body in get_bodies():
        print('Body id: {} | Name: {}: | Rigid: {} | Fixed: {}'.format(
            body, get_body_name(body), is_rigid_body(body), is_fixed_body(body)))
        for joint in get_joints(body):
            print('Joint id: {} | Name: {} | Type: {} | Circular: {} | Limits: {}'.format(
                joint, get_joint_name(body, joint), JOINT_TYPES[get_joint_type(body, joint)],
                is_circular(body, joint), get_joint_limits(body, joint)))
        print('Link id: {} | Name: {} | Mass: {}'.format(-1, get_base_name(body), get_mass(body)))
        for link in get_links(body):
            print('Link id: {} | Name: {} | Mass: {}'.format(link, get_link_name(body, link), get_mass(body, link)))
        print()

def is_rigid_body(body):
    for joint in get_joints(body):
        if is_movable(body, joint):
            return False
    return True

def is_fixed_body(body):
    return get_mass(body) == 0

#####################################

# Joints

JOINT_TYPES = {
    p.JOINT_REVOLUTE: 'revolute',
    p.JOINT_PRISMATIC: 'prismatic',
    p.JOINT_FIXED: 'fixed',
    p.JOINT_SPHERICAL: 'spherical',
    p.JOINT_PLANAR: 'planar'
}

def get_num_joints(body):
    return p.getNumJoints(body)

def get_joints(body):
    return list(range(get_num_joints(body)))

def get_joint(body, joint_or_name):
    if type(joint_or_name) is str:
        return joint_from_name(body, joint_or_name)
    return joint_or_name

def get_joint_name(body, joint):
    return p.getJointInfo(body, joint)[1].decode('UTF-8')

def get_joint_names(body):
    return [get_joint_name(body, joint) for joint in get_joints(body)]

def joint_from_name(body, name):
    for joint in get_joints(body):
        if get_joint_name(body, joint) == name:
            return joint
    raise ValueError(body, name)

def joints_from_names(body, names):
    return tuple(joint_from_name(body, name) for name in names)

def get_joint_position(body, joint):
    return p.getJointState(body, joint)[0]

def get_joint_positions(body, joints=None):
    return tuple(get_joint_position(body, joint) for joint in joints)

def set_joint_position(body, joint, value):
    p.resetJointState(body, joint, value)

def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        set_joint_position(body, joint, value)

def get_configuration(body):
    return get_joint_positions(body, get_movable_joints(body))

def set_configuration(body, values):
    set_joint_positions(body, get_movable_joints(body), values)

def get_joint_type(body, joint):
    return p.getJointInfo(body, joint)[2]

def is_movable(body, joint):
    return get_joint_type(body, joint) != p.JOINT_FIXED

def get_movable_joints(body): # 45 / 87 on pr2
    return [joint for joint in get_joints(body) if is_movable(body, joint)]

def joint_from_movable(body, index):
    return get_joints(body)[index]

def is_circular(body, joint):
    lower, upper = p.getJointInfo(body, joint)[8:10]
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

def get_joint_q_index(body, joint):
    return p.getJointInfo(body, joint)[3]

def get_joint_v_index(body, joint):
    return p.getJointInfo(body, joint)[4]

def get_joint_axis(body, joint):
    return p.getJointInfo(body, joint)[13]

def get_joint_parent_frame(body, joint):
    point = p.getJointInfo(body, joint)[14]
    euler = p.getJointInfo(body, joint)[15]
    return (point, quat_from_euler(euler))

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

BASE_LINK = -1
STATIC_MASS = 0

get_links = get_joints

def get_link_name(body, link):
    return p.getJointInfo(body, link)[12]

def get_link_parent(body, joint):
    return p.getJointInfo(body, joint)[16]

def link_from_name(body, name):
    for link in get_joints(body):
        if get_link_name(body, link) == name:
            return link
    raise ValueError(body, name)

def get_com_pose(body, link): # COM = center of mass
    return p.getLinkState(body, link)[0:2]

def get_inertial_pose(body, link):
    return p.getLinkState(body, link)[2:4]

def get_link_pose(body, link):
    # if set to 1 (or True), the Cartesian world position/orientation will be recomputed using forward kinematics.
    return p.getLinkState(body, link)[4:6]

def get_adjacent_links(body):
    adjacent = set()
    for link in get_links(body):
        parent = get_link_parent(body, link)
        adjacent.add((link, parent))
        #adjacent.add((parent, link))
    return adjacent


def get_adjacent_fixed_links(body):
    return filter(lambda item: not is_movable(body, item[0]), get_adjacent_links(body))


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

DynamicsInfo = namedtuple('DynamicsInfo', ['mass', 'lateral_friction',
                                           'local_inertia_diagonal', 'local_inertial_pos',  'local_inertial_orn',
                                           'restitution', 'rolling_friction', 'spinning_friction',
                                           'contact_damping', 'contact_stiffness'])

def get_dynamics_info(body, link=BASE_LINK):
    return DynamicsInfo(*p.getDynamicsInfo(body, link))

def get_mass(body, link=BASE_LINK):
    return get_dynamics_info(body, link).mass

#####################################

# Shapes

SHAPE_TYPES = {
    p.GEOM_SPHERE: 'sphere',
    p.GEOM_BOX: 'box',
    p.GEOM_CAPSULE: 'capsule',
    p.GEOM_CYLINDER: 'cylinder',
    p.GEOM_PLANE: 'plane',
    p.GEOM_MESH: 'mesh',
}

def create_box(w, l, h, mass=0, color=(1, 0, 0, 1)):
    half_extents = [w/2., l/2., h/2.]
    collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    if (color is None) or not has_gui():
        visual_id = -1
    else:
        visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id) # basePosition | baseOrientation
    # linkCollisionShapeIndices | linkVisualShapeIndices

def create_mesh():
    raise NotImplementedError()


def create_cylinder(radius, height):
    collision_id =  p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    raise NotImplementedError()


def create_capsule(radius, height):
    collision_id = p.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=height)
    raise NotImplementedError()

def create_sphere(radius, mass=0, color=(0, 0, 1, 1)):
    # mass = 0  => static
    collision_id = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    if (color is None) or not has_gui():
        visual_id = -1
    else:
        visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id) # basePosition | baseOrientation


def create_plane():
    collision_id = p.createVisualShape(p.GEOM_PLANE, normal=[])
    raise NotImplementedError()

def get_shape_data(body):
    return p.getVisualShapeData(body)

CollisionShapeData = namedtuple('CollisionShapeData', ['unique_id', 'link_index',
                                                       'geometry_type', 'dimensions', 'filename',
                                                       'local_frame_pos', 'local_frame_orn'])

def get_collision_data(body, link=-1): # 1 for the base
    return [CollisionShapeData(*tup) for tup in p.getCollisionShapeData(body, link)]

#####################################

# Bounding box

def get_lower_upper(body):
    return p.getAABB(body)

def get_center_extent(body):
    lower, upper = get_lower_upper(body)
    center = (np.array(lower) + upper) / 2
    extents = (np.array(upper) - lower)
    return center, extents

def aabb2d_from_aabb(aabb):
    (lower, upper) = aabb
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

def get_sample_fn(body, joints):
    def fn():
        values = []
        for joint in joints:
            limits = REVOLUTE_LIMITS if is_circular(body, joint) \
                else get_joint_limits(body, joint)
            values.append(np.random.uniform(*limits))
        return tuple(values)
    return fn

def get_difference_fn(body, joints):
    def fn(q2, q1):
        difference = []
        for joint, value2, value1 in zip(joints, q2, q1):
            difference.append(circular_difference(value2, value1)
                              if is_circular(body, joint) else (value2 - value1))
        return tuple(difference)
    return fn

def get_distance_fn(body, joints, weights=None):
    # TODO: custom weights and step sizes
    if weights is None:
        weights = 1*np.ones(len(joints))
    difference_fn = get_difference_fn(body, joints)
    def fn(q1, q2):
        diff = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, diff * diff))
    return fn

def get_refine_fn(body, joints, num_steps=0):
    difference_fn = get_difference_fn(body, joints)
    num_steps = num_steps + 1
    def fn(q1, q2):
        q = q1
        for i in range(num_steps):
            q = tuple((1. / (num_steps - i)) * np.array(difference_fn(q2, q)) + q)
            yield q
            # TODO: should wrap these joints
    return fn

def get_extend_fn(body, joints, resolutions=None):
    if resolutions is None:
        resolutions = 0.05*np.ones(len(joints))
    difference_fn = get_difference_fn(body, joints)
    def fn(q1, q2):
        steps = np.abs(np.divide(difference_fn(q2, q1), resolutions))
        refine_fn = get_refine_fn(body, joints, num_steps=int(np.max(steps)))
        return refine_fn(q1, q2)
    return fn

def sparsify_path(body, joints, path):
    if len(path) <= 2:
        return path
    difference_fn = get_difference_fn(body, joints)
    waypoints = [path[0]]
    last_difference = difference_fn(waypoints[-1], path[1])
    last_conf = path[1]
    for q in path[2:]:
        new_difference = difference_fn(waypoints[-1], q)
        #if np.allclose(last_difference, new_difference, atol=1e-3, rtol=0):
        #
        #last_difference = new_difference
        #last_conf = q
        # TODO: test if a scaling of itself
    return path

def plan_joint_motion(body, joints, end_conf, obstacles=None, direct=False, **kwargs):
    assert len(joints) == len(end_conf)

    sample_fn = get_sample_fn(body, joints)
    distance_fn = get_distance_fn(body, joints)
    extend_fn = get_extend_fn(body, joints)

    def collision_fn(q):
        if violates_limits(body, joints, q):
            return True
        set_joint_positions(body, joints, q)
        if obstacles is None:
            return env_collision(body)
        return any(pairwise_collision(body, obs) for obs in obstacles)

    start_conf = get_joint_positions(body, joints)
    if direct:
        return direct_path(start_conf, end_conf, extend_fn, collision_fn)
    return birrt(start_conf, end_conf, distance_fn,
                 sample_fn, extend_fn, collision_fn, **kwargs)

def plan_base_motion(body, end_conf, obstacles=None, direct=False, **kwargs):
    BASE_LIMITS = ([-2.5, -2.5, 0], [2.5, 2.5, 0])
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
        for i in range(n):
            q = tuple((1. / (n - i)) * np.array(difference_fn(q2, q)) + q)
            yield q
            # TODO: should wrap these joints

    def collision_fn(q):
        set_base_values(body, q)
        if obstacles is None:
            return env_collision(body)
        return any(pairwise_collision(body, obs) for obs in obstacles)

    start_conf = get_base_values(body)
    if direct:
        return direct_path(start_conf, end_conf, extend_fn, collision_fn)
    return birrt(start_conf, end_conf, distance_fn,
                 sample_fn, extend_fn, collision_fn, **kwargs)

#####################################

# Placements

def stable_z(body, surface):
    _, extent = get_center_extent(body)
    _, upper = get_lower_upper(surface)
    return (upper + extent/2)[2]

def is_placement(body, surface, epsilon=1e-2): # TODO: above / below
    top_aabb = get_lower_upper(body)
    bottom_aabb = get_lower_upper(surface)
    bottom_z_max = bottom_aabb[1][2]
    return (bottom_z_max <= top_aabb[0][2] <= (bottom_z_max + epsilon)) and \
           (aabb_contains(aabb2d_from_aabb(top_aabb), aabb2d_from_aabb(bottom_aabb)))

def sample_placement(top_body, bottom_body, max_attempts=50):
    bottom_aabb = get_lower_upper(bottom_body)
    for _ in range(max_attempts):
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

GraspInfo = namedtuple('GraspInfo', ['get_grasps', 'approach_pose'])
HoldingInfo = namedtuple('HoldingInfo', ['frame', 'grasp_pose', 'body'])

def end_effector_from_body(body_pose, grasp_pose):
    return multiply(body_pose, invert(grasp_pose))


def body_from_end_effector(end_effector_pose, grasp_pose):
    return multiply(end_effector_pose, grasp_pose)


def approach_from_grasp(approach_pose, end_effector_pose):
    return multiply(approach_pose, end_effector_pose)

#####################################

# Reachability

def sample_reachable_base(robot, point, reachable_range=(0.25, 1.0), max_attempts=50):
    for _ in range(max_attempts):
        radius = np.random.uniform(*reachable_range)
        x, y = radius*unit_from_theta(np.random.uniform(-np.pi, np.pi)) + point[:2]
        yaw = np.random.uniform(*REVOLUTE_LIMITS)
        base_values = (x, y, yaw)
        set_base_values(robot, base_values)
        return base_values
    return None


#####################################

# Constraints

def get_constraints():
    return list(range(p.getNumConstraints()))

def remove_constraint(constraint):
    p.removeConstraint(constraint)

ConstraintInfo = namedtuple('ConstraintInfo', ['parentBodyUniqueId', 'parentJointIndex',
                                               'childBodyUniqueId', 'childLinkIndex', 'constraintType',
                                               'jointAxis', 'jointPivotInParent', 'jointPivotInChild',
                                               'jointFrameOrientationParent', 'jointFrameOrientationChild', 'maxAppliedForce'])

def get_constraint_info(constraint):
    return [ConstraintInfo(*tup) for tup in p.getConstraintInfo(constraint)]

def fixed_constraint(parent_body, link1, child_body, link2):
    return p.createConstraint(parent_body, link1, child_body, link2,
                              p.JOINT_FIXED, jointAxis=[0]*3,  # JOINT_FIXED
                              parentFramePosition=[0]*3,
                              childFramePosition=[0]*3)

def grasp_constraint(body, robot, robot_link):
    body_link = -1
    body_pose = get_pose(body)
    end_effector_pose = get_link_pose(robot, robot_link)
    grasp_pose = multiply(invert(end_effector_pose), body_pose)
    point, quat = grasp_pose
    # TODO: can I do this when I'm not adjacent?
    # joint axis in local frame (ignored for JOINT_FIXED)
    #return p.createConstraint(robot, robot_link, body, body_link,
    #                          p.JOINT_FIXED, jointAxis=unit_point(),
    #                          parentFramePosition=unit_point(),
    #                          childFramePosition=point,
    #                          parentFrameOrientation=unit_quat(),
    #                          childFrameOrientation=quat)
    return p.createConstraint(robot, robot_link, body, body_link, # Both seem to work
                              p.JOINT_FIXED, jointAxis=unit_point(),
                              parentFramePosition=point,
                              childFramePosition=unit_point(),
                              parentFrameOrientation=quat,
                              childFrameOrientation=unit_quat())

def get_grasp_constraints():
    pass



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
    #kp = 1.0
    #kv = 0.3
    return p.setJointMotorControlArray(body, joints, p.POSITION_CONTROL, targetPositions=positions,
                                targetVelocities=[0.0] * len(joints)) #,
                                #positionGains=[kp] * len(joints),
                                #velocityGains=[kv] * len(joints),)
                                #forces=[])

def joint_controller(body, joints, target, max_time=None):
    iteration = 0
    while not np.allclose(get_joint_positions(body, joints), target, atol=1e-3, rtol=0):
        control_joints(body, joints, target)
        yield iteration
        iteration += 1

#####################################

class BodySaver(object): # TODO: with statements
    def __init__(self, body, pose=None):
        if pose is None:
            pose = get_pose(body)
        self.body = body
        self.pose = pose
        self.conf = get_configuration(body)
    def restore(self):
        set_configuration(self.body, self.conf)
        set_pose(self.body, self.pose)


class WorldSaver(object):
    def __init__(self):
        self.body_savers = [BodySaver(body) for body in get_bodies()]
    def restore(self):
        for body_saver in self.body_savers:
            body_saver.restore()
