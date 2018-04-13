from __future__ import print_function

import sys
import math
import time
import platform
from collections import defaultdict, deque, namedtuple
from itertools import product, combinations

# from future_builtins import map, filter
# from builtins import input # TODO - use future
try:
   input = raw_input
except NameError:
   pass

import numpy as np
import pybullet as p
import pickle
from transformations import quaternion_from_matrix

INF = np.inf
PI = np.pi
CIRCULAR_LIMITS = -PI, PI

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

def is_darwin(): # TODO: change loading accordingly
    return platform.system() == 'Darwin' # platform.release()

def write_pickle(filename, data):  # NOTE - cannot pickle lambda or nested functions
    with open(filename, 'wb') as f:
        pickle.dump(data, f)

def read_pickle(filename):
    with open(filename, 'rb') as f:
        return pickle.load(f)

#####################################

# Simulation

def load_model(model_file, pose=None, fixed_base=True):
    #p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
    #p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS
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
        disable_gravity()
    # TODO: wait until keypress

def simulate_for_duration(duration, dt=0):
    t0 = time.time()
    while (time.time() - t0) <= duration:
        step_simulation()
        time.sleep(dt)

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
    p.setGravity(0, 0, -9.8)

def disable_gravity():
    p.setGravity(0, 0, 0)

def step_simulation():
    p.stepSimulation()

def enable_real_time():
    p.setRealTimeSimulation(1)

def disable_real_time():
    p.setRealTimeSimulation(0)

def update_state():
    #disable_gravity()
    step_simulation()
    #for body in get_bodies():
    #    for link in get_links(body):
    #        # if set to 1 (or True), the Cartesian world position/orientation
    #        # will be recomputed using forward kinematics.
    #        get_link_state(body, link)
    #for body in get_bodies():
    #    get_pose(body)
    #    for joint in get_joints(body):
    #        get_joint_position(body, joint)
    #p.getKeyboardEvents()
    #p.getMouseEvents()

def reset_simulation():
    p.resetSimulation()

def get_camera():
    return p.getDebugVisualizerCamera()

def set_camera(yaw, pitch, distance, target_position=np.zeros(3)):
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

def get_bodies():
    return [p.getBodyUniqueId(i) for i in range(p.getNumBodies())]

BodyInfo = namedtuple('BodyInfo', ['base_name', 'body_name'])

def get_body_info(body):
    return BodyInfo(*p.getBodyInfo(body))

def get_base_name(body):
    return get_body_info(body).base_name

def get_body_name(body):
    return get_body_info(body).body_name

def has_body(name):
    try:
        body_from_name(name)
    except ValueError:
        return False
    return True

def body_from_name(name):
    for body in get_bodies():
        if get_body_name(body) == name:
            return body
    raise ValueError(name)

def remove_body(body):
    return p.removeBody(body)

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

def is_rigid_body(body):
    for joint in get_joints(body):
        if is_movable(body, joint):
            return False
    return True

def is_fixed_base(body):
    return get_mass(body) == STATIC_MASS

def dump_body(body):
    print('Body id: {} | Name: {} | Rigid: {} | Fixed: {}'.format(
        body, get_body_name(body), is_rigid_body(body), is_fixed_base(body)))
    for joint in get_joints(body):
        print('Joint id: {} | Name: {} | Type: {} | Circular: {} | Limits: {}'.format(
            joint, get_joint_name(body, joint), JOINT_TYPES[get_joint_type(body, joint)],
            is_circular(body, joint), get_joint_limits(body, joint)))
    print('Link id: {} | Name: {} | Mass: {}'.format(-1, get_base_name(body), get_mass(body)))
    for link in get_links(body):
        print('Link id: {} | Name: {} | Parent: {} | Mass: {}'.format(
            link, get_link_name(body, link), get_link_name(body, get_link_parent(body, link)),
            get_mass(body, link)))

def dump_world():
    for body in get_bodies():
        dump_body(body)
        print()

#####################################

# Joints

JOINT_TYPES = {
    p.JOINT_REVOLUTE: 'revolute',
    p.JOINT_PRISMATIC: 'prismatic',
    p.JOINT_FIXED: 'fixed',
    p.JOINT_SPHERICAL: 'spherical',
    p.JOINT_PLANAR: 'planar',
    p.JOINT_POINT2POINT: 'point2point',
    p.JOINT_GEAR: 'gear',
}

def get_num_joints(body):
    return p.getNumJoints(body)

def get_joints(body):
    return list(range(get_num_joints(body)))

def get_joint(body, joint_or_name):
    if type(joint_or_name) is str:
        return joint_from_name(body, joint_or_name)
    return joint_or_name

JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType',
                                     'qIndex', 'uIndex', 'flags',
                                     'jointDamping', 'jointFriction', 'jointLowerLimit', 'jointUpperLimit',
                                     'jointMaxForce', 'jointMaxVelocity', 'linkName', 'jointAxis',
                                     'parentFramePos', 'parentFrameOrn', 'parentIndex'])

def get_joint_info(body, joint):
    return JointInfo(*p.getJointInfo(body, joint))

def get_joint_name(body, joint):
    return get_joint_info(body, joint).jointName.decode('UTF-8')

#def get_joint_names(body):
#    return [get_joint_name(body, joint) for joint in get_joints(body)]

def joint_from_name(body, name):
    for joint in get_joints(body):
        if get_joint_name(body, joint) == name:
            return joint
    raise ValueError(body, name)

def has_joint(body, name):
    try:
        joint_from_name(body, name)
    except ValueError:
        return False
    return True

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
    return get_joint_info(body, joint).jointType

def is_movable(body, joint):
    return get_joint_type(body, joint) != p.JOINT_FIXED

def get_movable_joints(body): # 45 / 87 on pr2
    return [joint for joint in get_joints(body) if is_movable(body, joint)]

def joint_from_movable(body, index):
    return get_joints(body)[index]

def is_circular(body, joint):
    joint_info = get_joint_info(body, joint)
    if joint_info.jointType == p.JOINT_FIXED:
        return False
    return joint_info.jointUpperLimit < joint_info.jointLowerLimit

def get_joint_limits(body, joint):
    if is_circular(body, joint):
        return CIRCULAR_LIMITS
    joint_info = get_joint_info(body, joint)
    return joint_info.jointLowerLimit, joint_info.jointUpperLimit

def get_min_limit(body, joint):
    return get_joint_limits(body, joint)[0]

def get_max_limit(body, joint):
    return get_joint_limits(body, joint)[1]

def get_max_velocity(body, joint):
    return get_joint_info(body, joint).jointMaxVelocity

def get_joint_q_index(body, joint):
    return get_joint_info(body, joint).qIndex

def get_joint_v_index(body, joint):
    return get_joint_info(body, joint).uIndex

def get_joint_axis(body, joint):
    return get_joint_info(body, joint).jointAxis

def get_joint_parent_frame(body, joint):
    joint_info = get_joint_info(body, joint)
    point = joint_info.parentFramePos
    euler = joint_info.parentFrameOrn
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
    if link == BASE_LINK:
        return get_base_name(body)
    return get_joint_info(body, link).linkName

def get_link_parent(body, link):
    if link == BASE_LINK:
        return None
    return get_joint_info(body, link).parentIndex

def link_from_name(body, name):
    if name == get_base_name(body):
        return BASE_LINK
    for link in get_joints(body):
        if get_link_name(body, link) == name:
            return link
    raise ValueError(body, name)


def has_link(body, name):
    try:
        link_from_name(body, name)
    except ValueError:
        return False
    return True

LinkState = namedtuple('LinkState', ['linkWorldPosition', 'linkWorldOrientation',
                                     'localInertialFramePosition', 'localInertialFrameOrientation',
                                     'worldLinkFramePosition', 'worldLinkFrameOrientation'])

def get_link_state(body, link):
    return LinkState(*p.getLinkState(body, link))

def get_com_pose(body, link): # COM = center of mass
    link_state = get_link_state(body, link)
    return link_state.linkWorldPosition, link_state.linkWorldOrientation

def get_inertial_pose(body, link):
    link_state = get_link_state(body, link)
    return link_state.localInertialFramePosition, link_state.localInertialFrameOrientation

def get_link_pose(body, link):
    # if set to 1 (or True), the Cartesian world position/orientation will be recomputed using forward kinematics.
    link_state = get_link_state(body, link)
    return link_state.worldLinkFramePosition, link_state.worldLinkFrameOrientation

def get_all_link_parents(body):
    return {link: get_link_parent(body, link) for link in get_links(body)}

def get_all_link_children(body):
    children = {}
    for child, parent in get_all_link_parents(body).items():
        if parent not in children:
            children[parent] = []
        children[parent].append(child)
    return children

def get_link_children(body, link):
    children = get_all_link_children(body)
    return children.get(link, [])

def get_link_ancestors(body, link):
    parent = get_link_parent(body, link)
    if parent is None:
        return []
    return get_link_ancestors(body, parent) + [parent]

def get_joint_ancestors(body, link):
    return get_link_ancestors(body, link) + [link]

def get_movable_joint_ancestors(body, link):
    return filter(lambda j: is_movable(body, j), get_joint_ancestors(body, link))

def get_link_descendants(body, link):
    descendants = []
    for child in get_link_children(body, link):
        descendants.append(child)
        descendants += get_link_descendants(body, child)
    return descendants

def are_links_adjacent(body, link1, link2):
    return (get_link_parent(body, link1) == link2) or \
           (get_link_parent(body, link2) == link1)

def get_adjacent_links(body):
    adjacent = set()
    for link in get_links(body):
        parent = get_link_parent(body, link)
        adjacent.add((link, parent))
        #adjacent.add((parent, link))
    return adjacent

def get_adjacent_fixed_links(body):
    return filter(lambda item: not is_movable(body, item[0]),
                  get_adjacent_links(body))


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

def create_box(w, l, h, mass=STATIC_MASS, color=(1, 0, 0, 1)):
    half_extents = [w/2., l/2., h/2.]
    collision_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    if (color is None) or not has_gui():
        visual_id = -1
    else:
        visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents, rgbaColor=color)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id) # basePosition | baseOrientation
    # linkCollisionShapeIndices | linkVisualShapeIndices

def create_cylinder(radius, height, mass=STATIC_MASS, color=(0, 0, 1, 1)):
    collision_id =  p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    if (color is None) or not has_gui():
        visual_id = -1
    else:
        visual_id = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, height=height, rgbaColor=color)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id) # basePosition | baseOrientation

def create_capsule(radius, height, mass=STATIC_MASS, color=(0, 0, 1, 1)):
    collision_id = p.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=height)
    if (color is None) or not has_gui():
        visual_id = -1
    else:
        visual_id = p.createVisualShape(p.GEOM_CAPSULE, radius=radius, height=height, rgbaColor=color)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id) # basePosition | baseOrientation

def create_sphere(radius, mass=STATIC_MASS, color=(0, 0, 1, 1)):
    # mass = 0  => static
    collision_id = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    if (color is None) or not has_gui():
        visual_id = -1
    else:
        visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id) # basePosition | baseOrientation

def create_plane(normal=[0, 0, 1], mass=STATIC_MASS, color=(.5, .5, .5, 1)):
    collision_id = p.createCollisionShape(p.GEOM_PLANE, normal=normal)
    if (color is None) or not has_gui():
        visual_id = -1
    else:
        visual_id = p.createVisualShape(p.GEOM_PLANE, normal=normal, rgbaColor=color)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id) # basePosition | baseOrientation


def create_mesh(filename, scale=1):
    collision_id = p.createVisualShape(p.GEOM_MESH, fileName=filename, meshScale=scale*np.ones(3))
    raise NotImplementedError()

VisualShapeData = namedtuple('VisualShapeData', ['objectUniqueId', 'linkIndex',
                                                 'visualGeometryType', 'dimensions', 'meshAssetFileName',
                                                 'localVisualFrame_position', 'localVisualFrame_orientation',
                                                 'rgbaColor'])

def visual_shape_from_data(data):
    return p.createVisualShape(shapeType=data.visualGeometryType,
                                  radius=get_data_radius(data.visualGeometryType, data.dimensions),
                                  halfExtents=get_data_extents(data.visualGeometryType, data.dimensions),
                                  length=get_data_height(data.visualGeometryType, data.dimensions),
                                  fileName=data.meshAssetFileName,
                                  meshScale=get_data_scale(data.visualGeometryType, data.dimensions),
                                  planeNormal=get_data_normal(data.visualGeometryType, data.dimensions),
                                   rgbaColor=data.rgbaColor,
                                #specularColor=,
                                   visualFramePosition=data.localVisualFrame_position,
                                   visualFrameOrientation=data.localVisualFrame_orientation)

def get_visual_data(body, link=BASE_LINK):
    visual_data = [VisualShapeData(*tup) for tup in p.getVisualShapeData(body)]
    return filter(lambda d: d.linkIndex == link, visual_data)

CollisionShapeData = namedtuple('CollisionShapeData', ['object_unique_id', 'linkIndex',
                                                       'geometry_type', 'dimensions', 'filename',
                                                       'local_frame_pos', 'local_frame_orn'])

def collision_shape_from_data(data):
    return p.createCollisionShape(shapeType=data.geometry_type,
                                  radius=get_data_radius(data.geometry_type, data.dimensions),
                                  halfExtents=get_data_extents(data.geometry_type, data.dimensions),
                                  height=get_data_height(data.geometry_type, data.dimensions),
                                  fileName=data.filename,
                                  meshScale=get_data_scale(data.geometry_type, data.dimensions),
                                  planeNormal=get_data_normal(data.geometry_type, data.dimensions),
                                  collisionFramePosition=data.local_frame_pos,
                                  collisionFrameOrientation=data.local_frame_pos)
    #return p.createCollisionShapeArray()

def clone_body_editor(body):
    from pybullet_utils.urdfEditor import UrdfEditor
    editor = UrdfEditor()
    editor.initializeFromBulletBody(body, 0)
    #return editor.createMultiBody() # pybullet.error: createVisualShapeArray failed.
    return createMultiBody(editor)

def save_body(body, filename):
    from pybullet_utils.urdfEditor import UrdfEditor
    editor = UrdfEditor()
    editor.initializeFromBulletBody(body, 0)
    editor.saveUrdf(filename)

def clone_body(body, collision=False, visual=False):
    # TODO: names are not retained
    def clone_collision_shape(link=BASE_LINK):
        if not collision:
            return -1
        collision_data = get_collision_data(body, link)
        if collision_data:
            assert (len(collision_data) == 1)
            return collision_shape_from_data(collision_data[0])
        return -1

    def clone_visual_shape(link=BASE_LINK):
        if not visual or not has_gui():
            return -1
        visual_data = get_visual_data(body, link)
        if visual_data:
            assert (len(visual_data) == 1)
            return visual_shape_from_data(visual_data[0])
        return -1

    masses = []
    collision_shapes = []
    visual_shapes = []
    positions = []
    orientations = []
    inertial_positions = []
    inertial_orientations = []
    parent_indices = []
    joint_types = []
    joint_axes = []
    for link in get_links(body):
        joint_info = get_joint_info(body, link)
        dynamics_info = get_dynamics_info(body, link)
        masses.append(dynamics_info.mass)
        collision_shapes.append(clone_collision_shape(link))
        visual_shapes.append(clone_visual_shape(link))
        positions.append(joint_info.parentFramePos)
        orientations.append(joint_info.parentFrameOrn)
        inertial_positions.append(dynamics_info.local_inertial_pos)
        inertial_orientations.append(dynamics_info.local_inertial_orn)
        parent_indices.append(joint_info.parentIndex)
        joint_types.append(joint_info.jointType)
        joint_axes.append(joint_info.jointAxis)
    # print(masses,
    # collision_shapes,
    # visual_shapes,
    # positions,
    # orientations,
    # inertial_positions,
    # inertial_orientations,
    # parent_indices,
    # joint_types,
    # joint_axes)

    base_dynamics_info = get_dynamics_info(body)
    return p.createMultiBody(baseMass=base_dynamics_info.mass,
                             baseCollisionShapeIndex=clone_collision_shape(),
                             baseVisualShapeIndex=clone_visual_shape(),
                             basePosition=get_point(body),
                             baseOrientation=get_quat(body),
                             baseInertialFramePosition=base_dynamics_info.local_inertial_pos,
                             baseInertialFrameOrientation=base_dynamics_info.local_inertial_orn,
                             linkMasses=masses,
                             linkCollisionShapeIndices=collision_shapes,
                             linkVisualShapeIndices=visual_shapes,
                             linkPositions=positions,
                             linkOrientations=orientations,
                             linkInertialFramePositions=inertial_positions,
                             linkInertialFrameOrientations=inertial_orientations,
                             linkParentIndices=parent_indices,
                             linkJointTypes=joint_types,
                             linkJointAxis=joint_axes,
                             useMaximalCoordinates=0)

def clone_skeleton(body):
    masses = []
    collision_shapes = []
    visual_shapes = []
    positions = []
    orientations = []
    inertial_positions = []
    inertial_orientations = []
    parent_indices = []
    joint_types = []
    joint_axes = []
    for link in get_links(body):
        joint_info = get_joint_info(body, link)
        masses.append(0)
        collision_shapes.append(-1)
        visual_shapes.append(-1)
        positions.append(joint_info.parentFramePos)
        orientations.append(joint_info.parentFrameOrn)
        inertial_positions.append(unit_point())
        inertial_orientations.append(unit_quat())
        parent_indices.append(joint_info.parentIndex)
        joint_types.append(joint_info.jointType)
        joint_axes.append(joint_info.jointAxis)

    return p.createMultiBody(baseMass=0,
                             baseCollisionShapeIndex=-1, baseVisualShapeIndex=-1,
                             basePosition=unit_point(), baseOrientation=unit_quat(),
                             baseInertialFramePosition=unit_point(), baseInertialFrameOrientation=unit_quat(),
                             linkMasses=masses,
                             linkCollisionShapeIndices=collision_shapes, linkVisualShapeIndices=visual_shapes,
                             linkPositions=positions, linkOrientations=orientations,
                             linkInertialFramePositions=inertial_positions, linkInertialFrameOrientations=inertial_orientations,
                             linkParentIndices=parent_indices,
                             linkJointTypes=joint_types, linkJointAxis=joint_axes)

def get_collision_data(body, link=BASE_LINK):
    return [CollisionShapeData(*tup) for tup in p.getCollisionShapeData(body, link)]

def get_data_extents(geometry_type, dimensions):
    """
    depends on geometry type:
    for GEOM_BOX: extents,
    for GEOM_SPHERE dimensions[0] = radius,
    for GEOM_CAPSULE and GEOM_CYLINDER, dimensions[0] = height (length), dimensions[1] = radius.
    For GEOM_MESH, dimensions is the scaling factor.
    :return:
    """
    if geometry_type == p.GEOM_BOX:
        return dimensions
    return [1, 1, 1]

def get_data_radius(geometry_type, dimensions):
    if geometry_type == p.GEOM_SPHERE:
        return dimensions[0]
    if geometry_type in (p.GEOM_SPHERE, p.GEOM_CAPSULE):
        return dimensions[1]
    return 0.5

def get_data_height(geometry_type, dimensions):
    if geometry_type in (p.GEOM_SPHERE, p.GEOM_CAPSULE):
        return dimensions[0]
    return 1

def get_data_scale(geometry_type, dimensions):
    if geometry_type == p.GEOM_MESH:
        return dimensions
    return [1, 1, 1]

def get_data_normal(geometry_type, dimensions):
    if geometry_type == p.GEOM_PLANE:
        return dimensions
    return [0, 0, 1]

def set_color(body, color, link=BASE_LINK, shape_index=-1):
    """
    Experimental for internal use, recommended ignore shapeIndex or leave it -1.
    Intention was to let you pick a specific shape index to modify,
    since URDF (and SDF etc) can have more than 1 visual shape per link.
    This shapeIndex matches the list ordering returned by getVisualShapeData.
    :param body:
    :param link:
    :param shape_index:
    :return:
    """
    return p.changeVisualShape(body, link, rgbaColor=color)

#####################################

# Bounding box

def get_lower_upper(body):
    return p.getAABB(body)

get_aabb = get_lower_upper

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

def get_bodies_in_region(aabb):
    (lower, upper) = aabb
    return p.getOverlappingObjects(lower, upper)

#####################################

# Collision

def contact_collision():
    p.stepSimulation()
    return len(p.getContactPoints()) != 0

ContactResult = namedtuple('ContactResult', ['contactFlag', 'bodyUniqueIdA', 'bodyUniqueIdB',
                                         'linkIndexA', 'linkIndexB', 'positionOnA', 'positionOnB',
                                         'contactNormalOnB', 'contactDistance', 'normalForce'])

#def single_collision(body, max_distance=1e-3):
#    return len(p.getClosestPoints(body, max_distance=max_distance)) != 0

#MAX_DISTANCE = 1e-3
MAX_DISTANCE = 0

def pairwise_collision(body1, body2, max_distance=MAX_DISTANCE): # 10000
    # TODO: confirm that this doesn't just check the base link
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance)) != 0 # getContactPoints


def pairwise_link_collision(body1, link1, body2, link2, max_distance=MAX_DISTANCE): # 10000
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                                  linkIndexA=link1, linkIndexB=link2)) != 0 # getContactPoints

def single_collision(body1, **kwargs):
    for body2 in get_bodies():
        if (body1 != body2) and pairwise_collision(body1, body2, **kwargs):
            return True
    return False

def all_collision(**kwargs):
    bodies = get_bodies()
    for i in range(len(bodies)):
        for j in range(i+1, len(bodies)):
            if pairwise_collision(bodies[i], bodies[j], **kwargs):
                return True
    return False

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

#def filtered_self_collision(body, acceptable=tuple(), max_distance=0):
#    contacts = p.getClosestPoints(body, body, max_distance) # -1 is the base
#    for (_, link1), (_, link2), _ in map(get_contact_links, contacts):
#        if (link1 != link2) and (link1, link2) not in acceptable:
#            return True
#    return False

RayResult = namedtuple('RayResult', ['objectUniqueId', 'linkIndex',
                                     'hit_fraction', 'hit_position', 'hit_normal'])

def ray_collision(rays):
    ray_starts = [start for start, _ in rays]
    ray_ends = [start for _, end in rays]
    return [RayResult(*tup) for tup in p.rayTestBatch(ray_starts, ray_ends)]
    #return RayResult(*p.rayTest(start, end))

#####################################

# Motion

def get_sample_fn(body, joints):
    def fn():
        values = []
        for joint in joints:
            limits = CIRCULAR_LIMITS if is_circular(body, joint) \
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

# def sparsify_path(body, joints, path):
#     if len(path) <= 2:
#         return path
#     difference_fn = get_difference_fn(body, joints)
#     waypoints = [path[0]]
#     last_difference = difference_fn(waypoints[-1], path[1])
#     last_conf = path[1]
#     for q in path[2:]:
#         new_difference = difference_fn(waypoints[-1], q)
#         #if np.allclose(last_difference, new_difference, atol=1e-3, rtol=0):
#         #
#         #last_difference = new_difference
#         #last_conf = q
#         # TODO: test if a scaling of itself
#     return path

def get_moving_links(body, moving_joints):
    moving_links = moving_joints[:]
    for link in moving_joints:
        moving_links += get_link_descendants(body, link)
    return list(set(moving_links))

def get_moving_pairs(body, moving_joints):
    """
    Check all fixed and moving pairs
    Do not check all fixed and fixed pairs
    Check all moving pairs with a common
    """
    moving_links = get_moving_links(body, moving_joints)
    for i in range(len(moving_links)):
        link1 = moving_links[i]
        ancestors1 = set(get_joint_ancestors(body, link1)) & set(moving_joints)
        for j in range(i+1, len(moving_links)):
            link2 = moving_links[j]
            ancestors2 = set(get_joint_ancestors(body, link2)) & set(moving_joints)
            if ancestors1 != ancestors2:
                #print(get_link_name(body, link1), ancestors1)
                #print(get_link_name(body, link2), ancestors2)
                #input('awefawef')
                yield link1, link2

def plan_joint_motion(body, joints, end_conf, obstacles=None, attachments=[],
                      self_collisions=True, disabled_collisions=set(), direct=False, **kwargs):
    assert len(joints) == len(end_conf)
    moving_bodies = [body] + [attachment.child for attachment in attachments]
    sample_fn = get_sample_fn(body, joints)
    distance_fn = get_distance_fn(body, joints)
    extend_fn = get_extend_fn(body, joints)
    # TODO: test self collision with the holding

    check_link_pairs = []
    if self_collisions:
        moving_links = get_moving_links(body, joints)
        fixed_links = list(set(get_links(body)) - set(moving_links))
        check_link_pairs = list(product(moving_links, fixed_links))
        if True:
            check_link_pairs += list(get_moving_pairs(body, joints))
        else:
            check_link_pairs += list(combinations(moving_links, 2))
        check_link_pairs = filter(lambda pair: not are_links_adjacent(body, *pair), check_link_pairs)
        check_link_pairs = filter(lambda pair: (pair not in disabled_collisions) and
                                               (pair[::-1] not in disabled_collisions), check_link_pairs)

    if obstacles is None:
        obstacles = list(set(get_bodies()) - set(moving_bodies))
    check_body_pairs = list(product(moving_bodies, obstacles)) #+ list(combinations(moving_bodies, 2))
    # TODO: maybe prune the link adjacent to the robot

    # TODO: end-effector constraints
    def collision_fn(q):
        if violates_limits(body, joints, q):
            return True
        set_joint_positions(body, joints, q)
        for attachment in attachments:
            attachment.assign()
        #if pairwise_collision(body, body):
        #    return True
        for link1, link2 in check_link_pairs:
            if pairwise_link_collision(body, link1, body, link2):
                print(get_link_name(body, link1), get_link_name(body, link2))
                return True
        return any(pairwise_collision(*pair) for pair in check_body_pairs)

    from motion_planners.rrt_connect import birrt, direct_path
    start_conf = get_joint_positions(body, joints)
    if collision_fn(start_conf):
        print("Warning: initial configuration is in collision")
        return None
    if collision_fn(end_conf):
        print("Warning: end configuration is in collision")
        return None
    if direct:
        return direct_path(start_conf, end_conf, extend_fn, collision_fn)
    return birrt(start_conf, end_conf, distance_fn,
                 sample_fn, extend_fn, collision_fn, **kwargs)

def plan_base_motion(body, end_conf, obstacles=None, direct=False,
                     base_limits=([-2.5, -2.5], [2.5, 2.5]),
                     weights=1*np.ones(3),
                     resolutions=0.05*np.ones(3),
                     **kwargs):
    def sample_fn():
        x, y = np.random.uniform(*base_limits)
        theta = np.random.uniform(*CIRCULAR_LIMITS)
        return (x, y, theta)

    def difference_fn(q2, q1):
        dx, dy = np.array(q2[:2]) - np.array(q1[:2])
        dtheta = circular_difference(q2[2], q1[2])
        return (dx, dy, dtheta)

    def distance_fn(q1, q2):
        difference = np.array(difference_fn(q2, q1))
        return np.sqrt(np.dot(weights, difference * difference))

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
            return single_collision(body)
        return any(pairwise_collision(body, obs) for obs in obstacles)

    from motion_planners.rrt_connect import birrt, direct_path
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
        theta = np.random.uniform(*CIRCULAR_LIMITS)
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

# Reachability

def sample_reachable_base(robot, point, reachable_range=(0.25, 1.0), max_attempts=50):
    for _ in range(max_attempts):
        radius = np.random.uniform(*reachable_range)
        x, y = radius*unit_from_theta(np.random.uniform(-np.pi, np.pi)) + point[:2]
        yaw = np.random.uniform(*CIRCULAR_LIMITS)
        base_values = (x, y, yaw)
        set_base_values(robot, base_values)
        return base_values
    return None

def uniform_pose_generator(robot, gripper_pose, **kwargs):
    point = point_from_pose(gripper_pose)
    while True:
        base_values = sample_reachable_base(robot, point)
        set_base_values(robot, base_values)
        yield get_pose(robot)

#####################################

# Constraints - applies forces when not satisfied

def get_constraints():
    """
    getConstraintUniqueId will take a serial index in range 0..getNumConstraints,  and reports the constraint unique id.
    Note that the constraint unique ids may not be contiguous, since you may remove constraints.
    """
    return [p.getConstraintUniqueId(i) for i in range(p.getNumConstraints())]

def remove_constraint(constraint):
    p.removeConstraint(constraint)

ConstraintInfo = namedtuple('ConstraintInfo', ['parentBodyUniqueId', 'parentJointIndex',
                                               'childBodyUniqueId', 'childLinkIndex', 'constraintType',
                                               'jointAxis', 'jointPivotInParent', 'jointPivotInChild',
                                               'jointFrameOrientationParent', 'jointFrameOrientationChild', 'maxAppliedForce'])

def get_constraint_info(constraint): # getConstraintState
    # TODO: four additional arguments
    return ConstraintInfo(*p.getConstraintInfo(constraint)[:11])

def get_fixed_constraints():
    fixed_constraints = []
    for constraint in get_constraints():
        constraint_info = get_constraint_info(constraint)
        if constraint_info.constraintType == p.JOINT_FIXED:
            fixed_constraints.append(constraint)
    return fixed_constraints

def add_fixed_constraint(body, robot, robot_link):
    body_link = BASE_LINK
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

def remove_fixed_constraint(body, robot, robot_link):
    for constraint in get_fixed_constraints():
        constraint_info = get_constraint_info(constraint)
        if (body == constraint_info.childBodyUniqueId) and \
                (BASE_LINK == constraint_info.childLinkIndex) and \
                (robot == constraint_info.parentBodyUniqueId) and \
                (robot_link == constraint_info.parentJointIndex):
            remove_constraint(constraint)

#####################################

# Grasps

GraspInfo = namedtuple('GraspInfo', ['get_grasps', 'approach_pose'])

class Attachment(object):
    def __init__(self, parent, parent_link, grasp_pose, child):
        self.parent = parent
        self.parent_link = parent_link
        self.grasp_pose = grasp_pose
        self.child = child
        #self.child_link = child_link # child_link=BASE_LINK
    def assign(self):
        parent_link_pose = get_link_pose(self.parent, self.parent_link)
        child_pose = body_from_end_effector(parent_link_pose, self.grasp_pose)
        set_pose(self.child, child_pose)
        return child_pose
    def __repr__(self):
        return '{}({},{})'.format(self.__class__.__name__, self.parent, self.child)

def body_from_end_effector(end_effector_pose, grasp_pose):
    """
    world_from_parent * parent_from_child = world_from_child
    """
    return multiply(end_effector_pose, grasp_pose)

def end_effector_from_body(body_pose, grasp_pose):
    """
    world_from_child * (parent_from_child)^(-1) = world_from_parent
    """
    return multiply(body_pose, invert(grasp_pose))

def approach_from_grasp(approach_pose, end_effector_pose):
    return multiply(approach_pose, end_effector_pose)

def get_grasp_pose(constraint):
    """
    Grasps are parent_from_child
    """
    constraint_info = get_constraint_info(constraint)
    assert(constraint_info.constraintType == p.JOINT_FIXED)
    joint_from_parent = (constraint_info.jointPivotInParent, constraint_info.jointFrameOrientationParent)
    joint_from_child = (constraint_info.jointPivotInChild, constraint_info.jointFrameOrientationChild)
    return multiply(invert(joint_from_parent), joint_from_child)

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

#####################################

def inverse_kinematics(robot, link, pose, max_iterations=200, tolerance=1e-3):
    (target_point, target_quat) = pose
    movable_joints = get_movable_joints(robot)
    for iterations in range(max_iterations):
        # TODO: stop is no progress
        # TODO: stop if collision or invalid joint limits
        kinematic_conf = p.calculateInverseKinematics(robot, link, target_point, target_quat)
        if (kinematic_conf is None) or any(map(math.isnan, kinematic_conf)):
            return None
        set_joint_positions(robot, movable_joints, kinematic_conf)
        link_point, link_quat = get_link_pose(robot, link)
        if np.allclose(link_point, target_point, atol=tolerance, rtol=0) and \
                np.allclose(link_quat, target_quat, atol=tolerance, rtol=0):
            break
    else:
        return None
    if violates_limits(robot, movable_joints, kinematic_conf):
        return None
    return kinematic_conf

def experimental_inverse_kinematics(robot, link, pose,
                       null_space=False, max_iterations=200, tolerance=1e-3):
    (point, quat) = pose
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
    #kinematic_conf = get_joint_positions(robot, movable_joints)
    for iterations in range(max_iterations): # 0.000863273143768 / iteration
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

#####

def createMultiBody(self, basePosition=[0,0,0],physicsClientId=0):
    #assume link[0] is base
    if (len(self.urdfLinks)==0):
        return -1

    base = self.urdfLinks[0]

    #v.tmp_collision_shape_ids=[]
    baseMass = base.urdf_inertial.mass
    baseCollisionShapeIndex = -1
    baseVisualShapeIndex = -1
    baseShapeTypeArray=[]
    baseRadiusArray=[]
    baseHalfExtentsArray=[]
    lengthsArray=[]
    fileNameArray=[]
    meshScaleArray=[]
    basePositionsArray=[]
    baseOrientationsArray=[]

    for v in base.urdf_collision_shapes:
        shapeType = v.geom_type
        baseShapeTypeArray.append(shapeType)
        baseHalfExtentsArray.append([0.5*v.geom_extents[0],0.5*v.geom_extents[1],0.5*v.geom_extents[2]])
        baseRadiusArray.append(v.geom_radius)
        lengthsArray.append(v.geom_length)
        fileNameArray.append(v.geom_meshfilename)
        meshScaleArray.append(v.geom_meshscale)
        basePositionsArray.append(v.origin_xyz)
        orn=p.getQuaternionFromEuler(v.origin_rpy)
        baseOrientationsArray.append(orn)

    if (len(baseShapeTypeArray)) and True:
        baseCollisionShapeIndex = p.createCollisionShapeArray(shapeTypes=baseShapeTypeArray,
                radii=baseRadiusArray,
                halfExtents=baseHalfExtentsArray,
                lengths=lengthsArray,
                fileNames=fileNameArray,
                meshScales=meshScaleArray,
                collisionFramePositions=basePositionsArray,
                collisionFrameOrientations=baseOrientationsArray,
                physicsClientId=physicsClientId)


        urdfVisuals = base.urdf_visual_shapes
        baseVisualShapeIndex = p.createVisualShapeArray(shapeTypes=[v.geom_type for v in urdfVisuals],
                                                   halfExtents=[[ext * 0.5 for ext in v.geom_extents] for v in urdfVisuals],
                                                   radii=[v.geom_radius for v in urdfVisuals],
                                                   lengths=[v.geom_length[0] for v in urdfVisuals],
                                                   fileNames=[v.geom_meshfilename for v in urdfVisuals],
                                                   meshScales=[v.geom_meshscale for v in urdfVisuals],
                                                   rgbaColors=[v.material_rgba for v in urdfVisuals],
                                                   visualFramePositions=[v.origin_xyz for v in urdfVisuals],
                                                   visualFrameOrientations=[v.origin_rpy for v in urdfVisuals],
                                                   physicsClientId=physicsClientId)
#                 urdfVisual = base.urdf_visual_shapes[0]
#                 baseVisualShapeIndex = p.createVisualShape(shapeType=urdfVisual.geom_type,
#                                                                    halfExtents=[ext * 0.5 for ext in urdfVisual.geom_extents],
#                                                                    radius=urdfVisual.geom_radius,
#                                                                    length=urdfVisual.geom_length[0],
#                                                                    fileName=urdfVisual.geom_meshfilename,
#                                                                    meshScale=urdfVisual.geom_meshscale,
#                                                                    rgbaColor=urdfVisual.material_rgba,
#                                                                    visualFramePosition=urdfVisual.origin_xyz,
#                                                                    visualFrameOrientation=urdfVisual.origin_rpy,
#                                                                    physicsClientId=physicsClientId)


    linkMasses=[]
    linkCollisionShapeIndices=[]
    linkVisualShapeIndices=[]
    linkPositions=[]
    linkOrientations=[]
    linkMeshScaleArray=[]
    linkInertialFramePositions=[]
    linkInertialFrameOrientations=[]
    linkParentIndices=[]
    linkJointTypes=[]
    linkJointAxis=[]

    for joint in self.urdfJoints:
        link = joint.link
        linkMass = link.urdf_inertial.mass
        linkCollisionShapeIndex=-1
        linkVisualShapeIndex=-1
        linkPosition=[0,0,0]
        linkOrientation=[0,0,0]
        linkInertialFramePosition=[0,0,0]
        linkInertialFrameOrientation=[0,0,0]
        linkParentIndex=self.linkNameToIndex[joint.parent_name]
        linkJointType=joint.joint_type
        linkJointAx = joint.joint_axis_xyz
        linkShapeTypeArray=[]
        linkRadiusArray=[]
        linkHalfExtentsArray=[]
        lengthsArray=[]
        fileNameArray=[]
        linkPositionsArray=[]
        linkOrientationsArray=[]

        for v in link.urdf_collision_shapes:
            shapeType = v.geom_type
            linkShapeTypeArray.append(shapeType)
            linkHalfExtentsArray.append([0.5*v.geom_extents[0],0.5*v.geom_extents[1],0.5*v.geom_extents[2]])
            linkRadiusArray.append(v.geom_radius)
            lengthsArray.append(v.geom_length)
            fileNameArray.append(v.geom_meshfilename)
            linkMeshScaleArray.append(v.geom_meshscale)
            linkPositionsArray.append(v.origin_xyz)
            linkOrientationsArray.append(p.getQuaternionFromEuler(v.origin_rpy))

        if (len(linkShapeTypeArray)) and True:
            linkCollisionShapeIndex = p.createCollisionShapeArray(shapeTypes=linkShapeTypeArray,
                radii=linkRadiusArray,
                halfExtents=linkHalfExtentsArray,
                lengths=lengthsArray,
                fileNames=fileNameArray,
                meshScales=linkMeshScaleArray,
                collisionFramePositions=linkPositionsArray,
                collisionFrameOrientations=linkOrientationsArray,
                physicsClientId=physicsClientId)

            urdfVisuals = link.urdf_visual_shapes
            linkVisualShapeIndex = p.createVisualShapeArray(shapeTypes=[v.geom_type for v in urdfVisuals],
                                               halfExtents=[[ext * 0.5 for ext in v.geom_extents] for v in urdfVisuals],
                                               radii=[v.geom_radius for v in urdfVisuals],
                                               lengths=[v.geom_length[0] for v in urdfVisuals],
                                               fileNames=[v.geom_meshfilename for v in urdfVisuals],
                                               meshScales=[v.geom_meshscale for v in urdfVisuals],
                                               rgbaColors=[v.material_rgba for v in urdfVisuals],
                                               visualFramePositions=[v.origin_xyz for v in urdfVisuals],
                                               visualFrameOrientations=[v.origin_rpy for v in urdfVisuals],
                                               physicsClientId=physicsClientId)

#                         linkVisualShapeIndex = p.createVisualShape(shapeType=urdfVisual.geom_type,
#                                                                    halfExtents=[ext * 0.5 for ext in urdfVisual.geom_extents],
#                                                                    radius=urdfVisual.geom_radius,
#                                                                    length=urdfVisual.geom_length[0],
#                                                                    fileName=urdfVisual.geom_meshfilename,
#                                                                    meshScale=urdfVisual.geom_meshscale,
#                                                                    rgbaColor=urdfVisual.material_rgba,
#                                                                    visualFramePosition=urdfVisual.origin_xyz,
#                                                                    visualFrameOrientation=urdfVisual.origin_rpy,
#                                                                    physicsClientId=physicsClientId)

        linkMasses.append(linkMass)
        linkCollisionShapeIndices.append(linkCollisionShapeIndex)
        linkVisualShapeIndices.append(linkVisualShapeIndex)
        linkPositions.append(joint.joint_origin_xyz)
        linkOrientations.append(p.getQuaternionFromEuler(joint.joint_origin_rpy))
        linkInertialFramePositions.append(link.urdf_inertial.origin_xyz)
        linkInertialFrameOrientations.append(p.getQuaternionFromEuler(link.urdf_inertial.origin_rpy))
        linkParentIndices.append(linkParentIndex)
        linkJointTypes.append(joint.joint_type)
        linkJointAxis.append(joint.joint_axis_xyz)
    obUid = p.createMultiBody(baseMass,\
                baseCollisionShapeIndex=baseCollisionShapeIndex,
                baseVisualShapeIndex=baseVisualShapeIndex,
                basePosition=basePosition,
                baseInertialFramePosition=base.urdf_inertial.origin_xyz,
                baseInertialFrameOrientation=base.urdf_inertial.origin_rpy,
                linkMasses=linkMasses,
                linkCollisionShapeIndices=linkCollisionShapeIndices,
                linkVisualShapeIndices=linkVisualShapeIndices,
                linkPositions=linkPositions,
                linkOrientations=linkOrientations,
                linkInertialFramePositions=linkInertialFramePositions,
                linkInertialFrameOrientations=linkInertialFrameOrientations,
                linkParentIndices=linkParentIndices,
                linkJointTypes=linkJointTypes,
                linkJointAxis=linkJointAxis,
                physicsClientId=physicsClientId)
    return obUid