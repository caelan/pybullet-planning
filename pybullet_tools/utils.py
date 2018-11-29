from __future__ import print_function

import math
import os
import pickle
import platform
import pybullet as p
import numpy as np
import sys
import time

from collections import defaultdict, deque, namedtuple
from itertools import product, combinations, count

from .transformations import quaternion_from_matrix, quaternion_slerp, unit_vector

directory = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(directory, '../motion'))
from motion_planners.rrt_connect import birrt, direct_path
#from ..motion.motion_planners.rrt_connect import birrt, direct_path

# from future_builtins import map, filter
# from builtins import input # TODO - use future
try:
   input = raw_input
except NameError:
   pass
user_input = input

INF = np.inf
PI = np.pi
CIRCULAR_LIMITS = -PI, PI
UNBOUNDED_LIMITS = -INF, INF
DEFAULT_TIME_STEP = 1./240. # seconds

#####################################

# Models

# Robots
DRAKE_IIWA_URDF = "models/drake/iiwa_description/urdf/iiwa14_polytope_collision.urdf"
KUKA_IIWA_URDF = "kuka_iiwa/model.urdf"
KUKA_IIWA_GRIPPER_SDF = "kuka_iiwa/kuka_with_gripper.sdf"
R2D2_URDF = "r2d2.urdf"
MINITAUR_URDF = "quadruped/minitaur.urdf"
HUMANOID_MJCF = "mjcf/humanoid.xml"
HUSKY_URDF = "husky/husky.urdf"

# Objects
KIVA_SHELF_SDF = "kiva_shelf/model.sdf"
SMALL_BLOCK_URDF = "models/drake/objects/block_for_pick_and_place.urdf"
BLOCK_URDF = "models/drake/objects/block_for_pick_and_place_mid_size.urdf"
SINK_URDF = 'models/sink.urdf'
STOVE_URDF = 'models/stove.urdf'

#####################################

# I/O

def is_darwin(): # TODO: change loading accordingly
    return platform.system() == 'Darwin' # platform.release()
    #return sys.platform == 'darwin'

def read(filename):
    with open(filename, 'r') as f:
        return f.read()

def write(filename, string):
    with open(filename, 'w') as f:
        f.write(string)

def write_pickle(filename, data):  # NOTE - cannot pickle lambda or nested functions
    with open(filename, 'wb') as f:
        pickle.dump(data, f)

def read_pickle(filename):
    with open(filename, 'rb') as f:
        return pickle.load(f)

def safe_remove(p):
    if os.path.exists(p):
        os.remove(p)

def ensure_dir(f):
    d = os.path.dirname(f)
    if not os.path.exists(d):
        os.makedirs(d)

#####################################

class Verbose(object):
    def __init__(self, verbose):
        self.verbose = verbose

    def __enter__(self):
        if not self.verbose:
            self.stdout = sys.stdout
            self.devnull = open(os.devnull, 'w')
            sys.stdout = self.devnull
        return self

    def __exit__(self, type, value, traceback):
        if not self.verbose:
            sys.stdout = self.stdout
            self.devnull.close()

# https://stackoverflow.com/questions/5081657/how-do-i-prevent-a-c-shared-library-to-print-on-stdout-in-python/14797594#14797594
# https://stackoverflow.com/questions/4178614/suppressing-output-of-module-calling-outside-library
# https://stackoverflow.com/questions/4675728/redirect-stdout-to-a-file-in-python/22434262#22434262


class HideOutput(object):
    '''
    A context manager that block stdout for its scope, usage:

    with HideOutput():
        os.system('ls -l')
    '''
    def __init__(self, *args, **kw):
        sys.stdout.flush()
        self._origstdout = sys.stdout
        self._oldstdout_fno = os.dup(sys.stdout.fileno())
        self._devnull = os.open(os.devnull, os.O_WRONLY)

    def __enter__(self):
        self._newstdout = os.dup(1)
        os.dup2(self._devnull, 1)
        os.close(self._devnull)
        sys.stdout = os.fdopen(self._newstdout, 'w')

    def __exit__(self, exc_type, exc_val, exc_tb):
        sys.stdout = self._origstdout
        sys.stdout.flush()
        os.dup2(self._oldstdout_fno, 1)

#####################################

# Savers

# TODO: move the saving to enter?

class Saver(object):
    def restore(self):
        raise NotImplementedError()
    def __enter__(self):
        pass
    def __exit__(self, type, value, traceback):
        self.restore()

class ClientSaver(Saver):
    def __init__(self, new_client=None):
        self.client = CLIENT
        if new_client is not None:
            set_client(new_client)

    def restore(self):
        set_client(self.client)


class StateSaver(Saver):
    def __init__(self):
        self.state = save_state()

    def restore(self):
        restore_state(self.state)


#####################################

class PoseSaver(Saver):
    def __init__(self, body):
        self.body = body
        self.pose = get_pose(self.body)

    def restore(self):
        set_pose(self.body, self.pose)

class ConfSaver(Saver):
    def __init__(self, body): #, joints):
        self.body = body
        self.conf = get_configuration(body)

    def restore(self):
        set_configuration(self.body, self.conf)

#####################################

class BodySaver(Saver):
    def __init__(self, body): #, pose=None):
        #if pose is None:
        #    pose = get_pose(body)
        self.body = body
        self.pose_saver = PoseSaver(body)
        self.conf_saver = ConfSaver(body)

    def restore(self):
        self.pose_saver.restore()
        self.conf_saver.restore()

class WorldSaver(Saver):
    def __init__(self):
        self.body_savers = [BodySaver(body) for body in get_bodies()]

    def restore(self):
        for body_saver in self.body_savers:
            body_saver.restore()

#####################################

# Simulation

CLIENT = 0
# TODO: keep track of all the clients?

def get_client(client=None):
    if client is None:
        return CLIENT
    return client

def set_client(client):
    global CLIENT
    CLIENT = client

BODIES = defaultdict(dict)
# TODO: update delete as well

URDFInfo = namedtuple('URDFInfo', ['name', 'path'])

def load_pybullet(filename, fixed_base=False):
    # fixed_base=False implies infinite base mass
    body = p.loadURDF(filename, useFixedBase=fixed_base, physicsClientId=CLIENT)
    BODIES[CLIENT][body] = URDFInfo(None, filename)
    return body

URDF_FLAGS = [p.URDF_USE_INERTIA_FROM_FILE,
              p.URDF_USE_SELF_COLLISION,
              p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT,
              p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS]


def get_model_path(rel_path): # TODO: add to search path
    directory = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(directory, '..', rel_path)

def load_model(rel_path, pose=None, fixed_base=True):
    # TODO: error with loadURDF when loading MESH visual and CYLINDER collision
    abs_path = get_model_path(rel_path)
    flags = 0 # by default, Bullet disables self-collision
    add_data_path()
    if abs_path.endswith('.urdf'):
        body = p.loadURDF(abs_path, useFixedBase=fixed_base, flags=flags, physicsClientId=CLIENT)
    elif abs_path.endswith('.sdf'):
        body = p.loadSDF(abs_path, physicsClientId=CLIENT)
    elif abs_path.endswith('.xml'):
        body = p.loadMJCF(abs_path, physicsClientId=CLIENT)
    elif abs_path.endswith('.bullet'):
        body = p.loadBullet(abs_path, physicsClientId=CLIENT)
    else:
        raise ValueError(abs_path)
    if pose is not None:
        set_pose(body, pose)
    BODIES[CLIENT][body] = URDFInfo(None, abs_path)
    return body

#####################################

class World(object):
    def __init__(self, client):
        self.client = client
        self.bodies = {}
    def activate(self):
        set_client(self.client)
    def load(self, path, name=None, fixed_base=False):
        body = p.loadURDF(path, useFixedBase=fixed_base, physicsClientId=self.client)
        self.bodies[body] = URDFInfo(name, path)
        return body
    def remove(self, body):
        del self.bodies[body]
        return p.removeBody(body, physicsClientId=CLIENT)
    def reset(self):
        p.resetSimulation(physicsClientId=self.client)
        self.bodies = {}
    # TODO: with statement
    def copy(self):
        raise NotImplementedError()
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, len(self.bodies))

#####################################

def elapsed_time(start_time):
    return time.time() - start_time

def wait_for_duration(duration): #, dt=0):
    t0 = time.time()
    while elapsed_time(t0) <= duration:
        disable_gravity()
    # TODO: wait until keypress

def simulate_for_duration(duration, dt=0):
    # WARNING: this simulates for a wall-clock duration rather than simulator duration
    t0 = time.time()
    while elapsed_time(t0) <= duration:
        step_simulation()
        time.sleep(dt)

def get_time_step():
    # {'gravityAccelerationX', 'useRealTimeSimulation', 'gravityAccelerationZ', 'numSolverIterations',
    # 'gravityAccelerationY', 'numSubSteps', 'fixedTimeStep'}
    return p.getPhysicsEngineParameters(physicsClientId=CLIENT)['fixedTimeStep']

def simulate_for_sim_duration(sim_duration, real_dt=0, frequency=INF):
    t0 = time.time()
    sim_dt = get_time_step()
    sim_time = 0
    last_print = 0
    while sim_time < sim_duration:
        if frequency < (sim_time - last_print):
            print('Sim time: {:.3f} | Real time: {:.3f}'.format(sim_time, elapsed_time(t0)))
            last_print = sim_time
        step_simulation()
        sim_time += sim_dt
        time.sleep(real_dt)

def wait_for_user():
    if is_darwin():
        # OS X doesn't multi-thread the OpenGL visualizer
        wait_for_interrupt()
    else:
        user_input('Press enter to continue')

def wait_for_interrupt(max_time=np.inf):
    """
    Hold Ctrl to move the camera as well as zoom
    """
    print('Press Ctrl-C to continue')
    try:
        wait_for_duration(max_time)
    except KeyboardInterrupt:
        pass
    finally:
        print()

# def wait_for_input(s=''):
#     print(s)
#     while True:
#         step_simulation()
#         line = sys.stdin.readline()
#         if line:
#             pass
#         #events = p.getKeyboardEvents() # TODO: only works when the viewer is in focus
#         #if events:
#         #    print(events)
#         # https://docs.python.org/2/library/select.html


def connect(use_gui=True, shadows=True):
    method = p.GUI if use_gui else p.DIRECT
    with HideOutput():
        sim_id = p.connect(method)
        #sim_id = p.connect(p.GUI, options="--opengl2") if use_gui else p.connect(p.DIRECT)
    if use_gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, False, physicsClientId=sim_id)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, shadows, physicsClientId=sim_id)
    #visualizer_options = {
    #    p.COV_ENABLE_WIREFRAME: 1,
    #    p.COV_ENABLE_SHADOWS: 0,
    #    p.COV_ENABLE_RENDERING: 0,
    #    p.COV_ENABLE_TINY_RENDERER: 1,
    #    p.COV_ENABLE_RGB_BUFFER_PREVIEW: 0,
    #    p.COV_ENABLE_DEPTH_BUFFER_PREVIEW: 0,
    #    p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW: 0,
    #    p.COV_ENABLE_VR_RENDER_CONTROLLERS: 0,
    #    p.COV_ENABLE_VR_PICKING: 0,
    #    p.COV_ENABLE_VR_TELEPORTING: 0,
    #}
    #for pair in visualizer_options.items():
    #    p.configureDebugVisualizer(*pair)
    return sim_id

def disconnect():
    # TODO: change CLIENT?
    with HideOutput():
        return p.disconnect(physicsClientId=CLIENT)

def is_connected():
    return p.getConnectionInfo(physicsClientId=CLIENT)['isConnected']

def get_connection(client=None):
    return p.getConnectionInfo(physicsClientId=get_client(client))['connectionMethod']

def has_gui(client=None):
    return get_connection(get_client(client)) == p.GUI

def get_data_path():
    import pybullet_data
    return pybullet_data.getDataPath()

def add_data_path():
    return p.setAdditionalSearchPath(get_data_path())

def enable_gravity():
    p.setGravity(0, 0, -9.8, physicsClientId=CLIENT)

def disable_gravity():
    p.setGravity(0, 0, 0, physicsClientId=CLIENT)

def step_simulation():
    p.stepSimulation(physicsClientId=CLIENT)

def enable_real_time():
    p.setRealTimeSimulation(1, physicsClientId=CLIENT)

def disable_real_time():
    p.setRealTimeSimulation(0, physicsClientId=CLIENT)

def update_state():
    # TODO: this doesn't seem to automatically update still
    disable_gravity()
    #step_simulation()
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
    p.resetSimulation(physicsClientId=CLIENT)

def get_camera():
    return p.getDebugVisualizerCamera(physicsClientId=CLIENT)

def set_camera(yaw, pitch, distance, target_position=np.zeros(3)):
    p.resetDebugVisualizerCamera(distance, yaw, pitch, target_position, physicsClientId=CLIENT)

def get_pitch(point):
    dx, dy, dz = point
    return np.math.atan2(dz, np.sqrt(dx ** 2 + dy ** 2))

def get_yaw(point):
    dx, dy, dz = point
    return np.math.atan2(dy, dx)

def set_camera_pose(camera_point, target_point=np.zeros(3)):
    delta_point = np.array(target_point) - np.array(camera_point)
    distance = np.linalg.norm(delta_point)
    yaw = get_yaw(delta_point) - np.pi/2 # TODO: hack
    pitch = get_pitch(delta_point)
    p.resetDebugVisualizerCamera(distance, math.degrees(yaw), math.degrees(pitch),
                                 target_point, physicsClientId=CLIENT)

def get_image(width=640, height=480):
    import scipy.misc
    rgb, depth, _ = p.getCameraImage(width, height, physicsClientId=CLIENT)[2:]
    print(rgb.shape) # (480, 640, 4)
    print(depth.shape) # (480, 640)
    scipy.misc.imsave('image.jpg', rgb[:,:,:3])
    # scipy.misc.toimage(image_array, cmin=0.0, cmax=...).save('outfile.jpg')
    return rgb # np.reshape(rgb, [width, height, 4])

def set_default_camera():
    set_camera(160, -35, 2.5, Point())

def save_state():
    return p.saveState(physicsClientId=CLIENT)

def restore_state(state_id):
    p.restoreState(stateId=state_id, clientServerId=CLIENT)

def save_bullet(filename):
    p.saveBullet(filename, physicsClientId=CLIENT)

def restore_bullet(filename):
    p.restoreState(fileName=filename, physicsClientId=CLIENT)

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

#def Pose2d(x=0., y=0., yaw=0.):
#    return np.array([x, y, yaw])

def invert(pose):
    (point, quat) = pose
    return p.invertTransform(point, quat)

def multiply(*poses):
    pose = poses[0]
    for next_pose in poses[1:]:
        pose = p.multiplyTransforms(pose[0], pose[1], *next_pose)
    return pose

def invert_quat(quat):
    pose = (unit_point(), quat)
    return quat_from_pose(invert(pose))

def multiply_quats(*quats):
    return quat_from_pose(multiply(*[(unit_point(), quat) for quat in quats]))

def unit_from_theta(theta):
    return np.array([np.cos(theta), np.sin(theta)])

def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler)

def euler_from_quat(quat):
    return p.getEulerFromQuaternion(quat)

def unit_point():
    return (0., 0., 0.)

def unit_quat():
    return quat_from_euler([0, 0, 0]) # [X,Y,Z,W]

def quat_from_vector_angle(vec, angle):
    return get_unit_vector(np.append(vec, [angle]))

def unit_pose():
    return (unit_point(), unit_quat())

def get_length(vec):
    return np.linalg.norm(vec)

def get_distance(p1, p2):
    return get_length(np.array(p2) - np.array(p1))

def angle_between(vec1, vec2):
    return np.math.acos(np.dot(vec1, vec2) / (get_length(vec1) *  get_length(vec2)))

def get_unit_vector(vec):
    norm = get_length(vec)
    if norm == 0:
        return vec
    return np.array(vec) / norm

def z_rotation(theta):
    return quat_from_euler([0, 0, theta])

def matrix_from_quat(quat):
    return np.array(p.getMatrixFromQuaternion(quat, physicsClientId=CLIENT)).reshape(3, 3)

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

def quat_angle_between(quat0, quat1): # quaternion_slerp
    q0 = unit_vector(quat0[:4])
    q1 = unit_vector(quat1[:4])
    d = np.dot(q0, q1)
    angle = math.acos(d)
    return angle

def all_between(lower_limits, values, upper_limits):
    assert len(lower_limits) == len(values)
    assert len(values) == len(upper_limits)
    return np.less_equal(lower_limits, values).all() and np.less_equal(values, upper_limits).all()

#####################################

# Bodies

def get_bodies():
    return [p.getBodyUniqueId(i, physicsClientId=CLIENT)
            for i in range(p.getNumBodies(physicsClientId=CLIENT))]

BodyInfo = namedtuple('BodyInfo', ['base_name', 'body_name'])

def get_body_info(body):
    return BodyInfo(*p.getBodyInfo(body, physicsClientId=CLIENT))

def get_base_name(body):
    return get_body_info(body).base_name.decode(encoding='UTF-8')

def get_body_name(body):
    return get_body_info(body).body_name.decode(encoding='UTF-8')

def get_name(body):
    name = get_body_name(body)
    if name == '':
        name = 'body'
    return '{}{}'.format(name, int(body))

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
    return p.removeBody(body, physicsClientId=CLIENT)

def get_pose(body):
    return p.getBasePositionAndOrientation(body, physicsClientId=CLIENT)
    #return np.concatenate([point, quat])

def get_point(body):
    return get_pose(body)[0]

def get_quat(body):
    return get_pose(body)[1] # [x,y,z,w]

def get_base_values(body):
    return base_values_from_pose(get_pose(body))

def set_pose(body, pose):
    (point, quat) = pose
    p.resetBasePositionAndOrientation(body, point, quat, physicsClientId=CLIENT)

def set_point(body, point):
    set_pose(body, (point, get_quat(body)))

def set_quat(body, quat):
    set_pose(body, (get_point(body), quat))

def pose_from_pose2d(pose2d):
    x, y, theta = pose2d
    return Pose(Point(x=x, y=y), Euler(yaw=theta))

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
        if is_movable(body, joint):
            print('Joint id: {} | Name: {} | Type: {} | Circular: {} | Limits: {}'.format(
                joint, get_joint_name(body, joint), JOINT_TYPES[get_joint_type(body, joint)],
                is_circular(body, joint), get_joint_limits(body, joint)))
    print('Link id: {} | Name: {} | Mass: {}'.format(-1, get_base_name(body), get_mass(body)))
    for link in get_links(body):
        print('Link id: {} | Name: {} | Parent: {} | Mass: {}'.format(
            link, get_link_name(body, link), get_link_name(body, get_link_parent(body, link)),
            get_mass(body, link)))
        #print(get_joint_parent_frame(body, link))
        #print(map(get_data_geometry, get_visual_data(body, link)))
        #print(map(get_data_geometry, get_collision_data(body, link)))

def dump_world():
    for body in get_bodies():
        dump_body(body)
        print()

#####################################

# Joints

JOINT_TYPES = {
    p.JOINT_REVOLUTE: 'revolute', # 0
    p.JOINT_PRISMATIC: 'prismatic', # 1
    p.JOINT_SPHERICAL: 'spherical', # 2
    p.JOINT_PLANAR: 'planar', # 3
    p.JOINT_FIXED: 'fixed', # 4
    p.JOINT_POINT2POINT: 'point2point', # 5
    p.JOINT_GEAR: 'gear', # 6
}

def get_num_joints(body):
    return p.getNumJoints(body, physicsClientId=CLIENT)

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
    return JointInfo(*p.getJointInfo(body, joint, physicsClientId=CLIENT))

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

JointState = namedtuple('JointState', ['jointPosition', 'jointVelocity',
                                     'jointReactionForces', 'appliedJointMotorTorque'])

def get_joint_state(body, joint):
    return JointState(*p.getJointState(body, joint, physicsClientId=CLIENT))

def get_joint_position(body, joint):
    return get_joint_state(body, joint).jointPosition

def get_joint_torque(body, joint):
    return get_joint_state(body, joint).appliedJointMotorTorque

def get_joint_positions(body, joints): # joints=None):
    return tuple(get_joint_position(body, joint) for joint in joints)

def set_joint_position(body, joint, value):
    p.resetJointState(body, joint, value, physicsClientId=CLIENT)

def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        set_joint_position(body, joint, value)

def get_configuration(body):
    return get_joint_positions(body, get_movable_joints(body))

def set_configuration(body, values):
    set_joint_positions(body, get_movable_joints(body), values)

def get_full_configuration(body):
    # Cannot alter fixed joints
    return get_joint_positions(body, get_joints(body))

def get_joint_type(body, joint):
    return get_joint_info(body, joint).jointType

def is_movable(body, joint):
    return get_joint_type(body, joint) != p.JOINT_FIXED

def prune_fixed_joints(body, joints):
    return [joint for joint in joints if is_movable(body, joint)]

def get_movable_joints(body): # 45 / 87 on pr2
    return prune_fixed_joints(body, get_joints(body))

def joint_from_movable(body, index):
    return get_joints(body)[index]

def movable_from_joints(body, joints):
    movable_from_original = {o: m for m, o in enumerate(get_movable_joints(body))}
    return [movable_from_original[joint] for joint in joints]

def is_circular(body, joint):
    joint_info = get_joint_info(body, joint)
    if joint_info.jointType == p.JOINT_FIXED:
        return False
    return joint_info.jointUpperLimit < joint_info.jointLowerLimit

def get_joint_limits(body, joint):
    # TODO: make a version for several joints?
    if is_circular(body, joint):
        # TODO: return UNBOUNDED_LIMITS
        return CIRCULAR_LIMITS
    joint_info = get_joint_info(body, joint)
    return joint_info.jointLowerLimit, joint_info.jointUpperLimit

def get_min_limit(body, joint):
    # TODO: rename to min_position
    return get_joint_limits(body, joint)[0]

def get_max_limit(body, joint):
    return get_joint_limits(body, joint)[1]

def get_max_velocity(body, joint):
    return get_joint_info(body, joint).jointMaxVelocity

def get_max_force(body, joint):
    return get_joint_info(body, joint).jointMaxForce

def get_joint_q_index(body, joint):
    return get_joint_info(body, joint).qIndex

def get_joint_v_index(body, joint):
    return get_joint_info(body, joint).uIndex

def get_joint_axis(body, joint):
    return get_joint_info(body, joint).jointAxis

def get_joint_parent_frame(body, joint):
    joint_info = get_joint_info(body, joint)
    return joint_info.parentFramePos, joint_info.parentFrameOrn

def violates_limit(body, joint, value):
    if is_circular(body, joint):
        return False
    lower, upper = get_joint_limits(body, joint)
    return (value < lower) or (upper < value)

def violates_limits(body, joints, values):
    return any(violates_limit(body, joint, value) for joint, value in zip(joints, values))

def wrap_position(body, joint, position):
    if is_circular(body, joint):
        return wrap_angle(position)
    return position

def wrap_positions(body, joints, positions):
    assert len(joints) == len(positions)
    return [wrap_position(body, joint, position)
            for joint, position in zip(joints, positions)]

def get_custom_limits(body, joints, custom_limits={}, circular_limits=UNBOUNDED_LIMITS):
    joint_limits = []
    for joint in joints:
        if joint in custom_limits:
            joint_limits.append(custom_limits[joint])
        elif is_circular(body, joint):
            joint_limits.append(circular_limits)
        else:
            joint_limits.append(get_joint_limits(body, joint))
    return zip(*joint_limits)

#####################################

# Links

BASE_LINK = -1
STATIC_MASS = 0

get_num_links = get_num_joints
get_links = get_joints # Does not include BASE_LINK

def get_all_links(body):
    return [BASE_LINK] + list(get_links(body))

def get_link_name(body, link):
    if link == BASE_LINK:
        return get_base_name(body)
    return get_joint_info(body, link).linkName.decode('UTF-8')

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
    # computeLinkVelocity | computeForwardKinematics
    return LinkState(*p.getLinkState(body, link, physicsClientId=CLIENT))

def get_com_pose(body, link): # COM = center of mass
    link_state = get_link_state(body, link)
    return link_state.linkWorldPosition, link_state.linkWorldOrientation

def get_link_inertial_pose(body, link):
    link_state = get_link_state(body, link)
    return link_state.localInertialFramePosition, link_state.localInertialFrameOrientation

def get_link_pose(body, link):
    if link == BASE_LINK:
        return get_pose(body)
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
    return prune_fixed_joints(body, get_joint_ancestors(body, link))

def get_link_descendants(body, link):
    descendants = []
    for child in get_link_children(body, link):
        descendants.append(child)
        descendants.extend(get_link_descendants(body, child))
    return descendants

def get_link_subtree(body, link):
    return [link] + get_link_descendants(body, link)

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
    return list(filter(lambda item: not is_movable(body, item[0]),
                  get_adjacent_links(body)))


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
        fixed.update(product(cluster, cluster))
    return fixed

DynamicsInfo = namedtuple('DynamicsInfo', ['mass', 'lateral_friction',
                                           'local_inertia_diagonal', 'local_inertial_pos',  'local_inertial_orn',
                                           'restitution', 'rolling_friction', 'spinning_friction',
                                           'contact_damping', 'contact_stiffness'])

def get_dynamics_info(body, link=BASE_LINK):
    return DynamicsInfo(*p.getDynamicsInfo(body, link, physicsClientId=CLIENT))

def get_mass(body, link=BASE_LINK):
    return get_dynamics_info(body, link).mass

def get_joint_inertial_pose(body, joint):
    dynamics_info = get_dynamics_info(body, joint)
    return dynamics_info.local_inertial_pos, dynamics_info.local_inertial_orn

def get_local_link_pose(body, joint):
    parent_joint = get_link_parent(body, joint)

    #world_child = get_link_pose(body, joint)
    #world_parent = get_link_pose(body, parent_joint)
    ##return multiply(invert(world_parent), world_child)
    #return multiply(world_child, invert(world_parent))

    # https://github.com/bulletphysics/bullet3/blob/9c9ac6cba8118544808889664326fd6f06d9eeba/examples/pybullet/gym/pybullet_utils/urdfEditor.py#L169
    parent_com = get_joint_parent_frame(body, joint)
    tmp_pose = invert(multiply(get_joint_inertial_pose(body, joint), parent_com))
    parent_inertia = get_joint_inertial_pose(body, parent_joint)
    #return multiply(parent_inertia, tmp_pose) # TODO: why is this wrong...
    _, orn = multiply(parent_inertia, tmp_pose)
    pos, _ = multiply(parent_inertia, Pose(parent_com[0]))
    return (pos, orn)

#####################################

# Shapes

SHAPE_TYPES = {
    p.GEOM_SPHERE: 'sphere', # 2
    p.GEOM_BOX: 'box', # 3
    p.GEOM_CYLINDER: 'cylinder', # 4
    p.GEOM_MESH: 'mesh', # 5
    p.GEOM_PLANE: 'plane',  # 6
    p.GEOM_CAPSULE: 'capsule',  # 7
}

# TODO: clean this up to avoid repeated work

def get_box_geometry(width, length, height):
    return {
        'shapeType': p.GEOM_BOX,
        'halfExtents': [width/2., length/2., height/2.]
    }

def get_cylinder_geometry(radius, height):
    return {
        'shapeType': p.GEOM_CYLINDER,
        'radius': radius,
        'height': height,
    }

def get_sphere_geometry(radius):
    return {
        'shapeType': p.GEOM_SPHERE,
        'radius': radius,
    }

def get_capsule_geometry(radius, height):
    return {
        'shapeType': p.GEOM_CAPSULE,
        'radius': radius,
        'length': height,
    }

def get_mesh_geometry(path, scale=1.):
    return {
        'shapeType': p.GEOM_MESH,
        'fileName': path,
        'meshScale': scale*np.ones(3),
    }

NULL_ID = -1

def create_shape(geometry, pose=unit_pose(), color=(1, 0, 0, 1), specular=None):
    point, quat = pose
    collision_args = {
        'collisionFramePosition': point,
        'collisionFrameOrientation': quat,
        'physicsClientId': CLIENT,
    }
    collision_args.update(geometry)
    collision_id = p.createCollisionShape(**collision_args)

    if (color is None) or not has_gui():
    #if not has_gui():
        return collision_id, NULL_ID
    if 'height' in geometry: # TODO: pybullet bug
        geometry['length'] = geometry['height']
        del geometry['height']
    visual_args = {
        'rgbaColor': color,
        'visualFramePosition': point,
        'visualFrameOrientation': quat,
        'physicsClientId': CLIENT,
    }
    visual_args.update(geometry)
    if specular is not None:
        visual_args['specularColor'] = specular
    visual_id = p.createVisualShape(**visual_args)
    return collision_id, visual_id

def plural(word):
    exceptions = {'radius': 'radii'}
    if word in exceptions:
        return exceptions[word]
    if word.endswith('s'):
        return word
    return word + 's'

def create_shape_array(geoms, poses, colors=None):
    mega_geom = defaultdict(list)
    for geom in geoms:
        extended_geom = get_default_geometry()
        extended_geom.update(geom)
        if 'height' in extended_geom:
            extended_geom['length'] = extended_geom['height']
            del extended_geom['height']
        for key, value in extended_geom.items():
            mega_geom[plural(key)].append(value)

    collision_args = mega_geom.copy()
    for pose in poses:
        point, quat = pose
        collision_args['collisionFramePositions'].append(point)
        collision_args['collisionFrameOrientations'].append(quat)
    collision_id = p.createCollisionShapeArray(physicsClientId=CLIENT, **collision_args)
    if (colors is None) or not has_gui():
        return collision_id, NULL_ID

    visual_args = mega_geom.copy()
    for pose, color in zip(poses, colors):
        point, quat = pose
        visual_args['rgbaColors'].append(color)
        visual_args['visualFramePositions'].append(point)
        visual_args['visualFrameOrientations'].append(quat)
    visual_id = p.createVisualShapeArray(physicsClientId=CLIENT, **visual_args)
    return collision_id, visual_id

#####################################

def create_body(collision_id, visual_id, mass=STATIC_MASS):
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)


def create_box(w, l, h, mass=STATIC_MASS, color=(1, 0, 0, 1)):
    collision_id, visual_id = create_shape(get_box_geometry(w, l, h), color=color)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)
    # basePosition | baseOrientation
    # linkCollisionShapeIndices | linkVisualShapeIndices

def create_cylinder(radius, height, mass=STATIC_MASS, color=(0, 0, 1, 1)):
    collision_id, visual_id = create_shape(get_cylinder_geometry(radius, height), color=color)
    return create_body(collision_id, visual_id, mass=mass)

def create_capsule(radius, height, mass=STATIC_MASS, color=(0, 0, 1, 1)):
    # TODO: combine this
    collision_id = p.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=height, physicsClientId=CLIENT)
    if (color is None) or not has_gui():
        visual_id = -1
    else:
        visual_id = p.createVisualShape(p.GEOM_CAPSULE, radius=radius, height=height, rgbaColor=color, physicsClientId=CLIENT)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id, physicsClientId=CLIENT) # basePosition | baseOrientation

def create_sphere(radius, mass=STATIC_MASS, color=(0, 0, 1, 1)):
    # mass = 0  => static
    collision_id = p.createCollisionShape(p.GEOM_SPHERE, radius=radius, physicsClientId=CLIENT)
    if (color is None) or not has_gui():
        visual_id = -1
    else:
        visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color, physicsClientId=CLIENT)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id, physicsClientId=CLIENT) # basePosition | baseOrientation

def create_plane(normal=[0, 0, 1], mass=STATIC_MASS, color=(.5, .5, .5, 1)):
    collision_id = p.createCollisionShape(p.GEOM_PLANE, normal=normal, physicsClientId=CLIENT)
    if (color is None) or not has_gui():
        visual_id = -1
    else:
        visual_id = p.createVisualShape(p.GEOM_PLANE, normal=normal, rgbaColor=color, physicsClientId=CLIENT)
    return p.createMultiBody(baseMass=mass, baseCollisionShapeIndex=collision_id,
                             baseVisualShapeIndex=visual_id, physicsClientId=CLIENT) # basePosition | baseOrientation

def create_obj(path, scale=1., mass=STATIC_MASS, color=(.5, .5, .5, 1.)):
    collision_id, visual_id = create_shape(get_mesh_geometry(path, scale=scale), color=color)
    return create_body(collision_id, visual_id, mass=mass)


mesh_count = count()
MESH_DIR = 'temp/'

def create_mesh(mesh, **kwargs):
    # http://people.sc.fsu.edu/~jburkardt/data/obj/obj.html
    # TODO: read OFF / WRL / OBJ files
    # TODO: maintain dict to file
    ensure_dir(MESH_DIR)
    path = os.path.join(MESH_DIR, 'mesh{}.obj'.format(next(mesh_count)))
    write(path, obj_file_from_mesh(mesh))
    return create_obj(path, **kwargs)
    #safe_remove(path) # TODO: removing might delete mesh?

#####################################

VisualShapeData = namedtuple('VisualShapeData', ['objectUniqueId', 'linkIndex',
                                                 'visualGeometryType', 'dimensions', 'meshAssetFileName',
                                                 'localVisualFrame_position', 'localVisualFrame_orientation',
                                                 'rgbaColor'])

def visual_shape_from_data(data, client):
    if (data.visualGeometryType == p.GEOM_MESH) and (data.meshAssetFileName == 'unknown_file'):
        return -1
    # visualFramePosition: translational offset of the visual shape with respect to the link
    # visualFrameOrientation: rotational offset (quaternion x,y,z,w) of the visual shape with respect to the link frame
    pose = (data.localVisualFrame_position, data.localVisualFrame_orientation)
    inertial_pose = get_joint_inertial_pose(data.objectUniqueId, data.linkIndex)
    point, quat = multiply(invert(inertial_pose), pose)
    return p.createVisualShape(shapeType=data.visualGeometryType,
                               radius=get_data_radius(data),
                               halfExtents=np.array(get_data_extents(data))/2,
                               length=get_data_height(data), # TODO: pybullet bug
                               fileName=data.meshAssetFileName,
                               meshScale=get_data_scale(data),
                               planeNormal=get_data_normal(data),
                               rgbaColor=data.rgbaColor,
                               #specularColor=,
                               visualFramePosition=point,
                               visualFrameOrientation=quat,
                               physicsClientId=client)

def get_visual_data(body, link=BASE_LINK):
    visual_data = [VisualShapeData(*tup) for tup in p.getVisualShapeData(body, physicsClientId=CLIENT)]
    return list(filter(lambda d: d.linkIndex == link, visual_data))

# object_unique_id and linkIndex seem to be noise
CollisionShapeData = namedtuple('CollisionShapeData', ['object_unique_id', 'linkIndex',
                                                       'geometry_type', 'dimensions', 'filename',
                                                       'local_frame_pos', 'local_frame_orn'])

def collision_shape_from_data(data, body, link, client):
    if (data.geometry_type == p.GEOM_MESH) and (data.filename == 'unknown_file'):
        return -1
    pose = (data.local_frame_pos, data.local_frame_orn)
    pose = multiply(invert(get_joint_inertial_pose(body, link)), pose)
    point, quat = pose
    # TODO: the visual data seems affected by the collision data
    return p.createCollisionShape(shapeType=data.geometry_type,
                                  radius=get_data_radius(data),
                                  # halfExtents=get_data_extents(data.geometry_type, data.dimensions),
                                  halfExtents=np.array(get_data_extents(data)) / 2,
                                  height=get_data_height(data),
                                  fileName=data.filename.decode(encoding='UTF-8'),
                                  meshScale=get_data_scale(data),
                                  planeNormal=get_data_normal(data),
                                  collisionFramePosition=point,
                                  collisionFrameOrientation=quat,
                                  physicsClientId=client)
    #return p.createCollisionShapeArray()

def clone_visual_shape(body, link, client):
    if not has_gui(client):
        return -1
    visual_data = get_visual_data(body, link)
    if visual_data:
        assert (len(visual_data) == 1)
        return visual_shape_from_data(visual_data[0], client)
    return -1

def clone_collision_shape(body, link, client):
    collision_data = get_collision_data(body, link)
    if collision_data:
        assert (len(collision_data) == 1)
        # TODO: can do CollisionArray
        return collision_shape_from_data(collision_data[0], body, link, client)
    return -1

def clone_body(body, links=None, collision=True, visual=True, client=None):
    # TODO: names are not retained
    # TODO: error with createMultiBody link poses on PR2
    # localVisualFrame_position: position of local visual frame, relative to link/joint frame
    # localVisualFrame orientation: orientation of local visual frame relative to link/joint frame
    # parentFramePos: joint position in parent frame
    # parentFrameOrn: joint orientation in parent frame
    client = get_client(client)
    if links is None:
        links = get_links(body)
    #movable_joints = [joint for joint in links if is_movable(body, joint)]
    new_from_original = {}
    base_link = get_link_parent(body, links[0]) if links else BASE_LINK
    new_from_original[base_link] = -1

    masses = []
    collision_shapes = []
    visual_shapes = []
    positions = [] # list of local link positions, with respect to parent
    orientations = [] # list of local link orientations, w.r.t. parent
    inertial_positions = [] # list of local inertial frame pos. in link frame
    inertial_orientations = [] # list of local inertial frame orn. in link frame
    parent_indices = []
    joint_types = []
    joint_axes = []
    for i, link in enumerate(links):
        new_from_original[link] = i
        joint_info = get_joint_info(body, link)
        dynamics_info = get_dynamics_info(body, link)
        masses.append(dynamics_info.mass)
        collision_shapes.append(clone_collision_shape(body, link, client) if collision else -1)
        visual_shapes.append(clone_visual_shape(body, link, client) if visual else -1)
        point, quat = get_local_link_pose(body, link)
        positions.append(point)
        orientations.append(quat)
        inertial_positions.append(dynamics_info.local_inertial_pos)
        inertial_orientations.append(dynamics_info.local_inertial_orn)
        parent_indices.append(new_from_original[joint_info.parentIndex] + 1) # TODO: need the increment to work
        joint_types.append(joint_info.jointType)
        joint_axes.append(joint_info.jointAxis)
    # https://github.com/bulletphysics/bullet3/blob/9c9ac6cba8118544808889664326fd6f06d9eeba/examples/pybullet/gym/pybullet_utils/urdfEditor.py#L169

    base_dynamics_info = get_dynamics_info(body, base_link)
    base_point, base_quat = get_link_pose(body, base_link)
    new_body = p.createMultiBody(baseMass=base_dynamics_info.mass,
                                 baseCollisionShapeIndex=clone_collision_shape(body, base_link, client) if collision else -1,
                                 baseVisualShapeIndex=clone_visual_shape(body, base_link, client) if visual else -1,
                                 basePosition=base_point,
                                 baseOrientation=base_quat,
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
                                 physicsClientId=client)
    #set_configuration(new_body, get_joint_positions(body, movable_joints)) # Need to use correct client
    for joint, value in zip(range(len(links)), get_joint_positions(body, links)):
        # TODO: check if movable?
        p.resetJointState(new_body, joint, value, physicsClientId=client)
    return new_body

def clone_world(client=None, exclude=[]):
    mapping = {}
    for body in get_bodies():
        if body not in exclude:
            new_body = clone_body(body, collision=True, visual=True, client=client)
            mapping[body] = new_body
    return mapping

#####################################

def get_collision_data(body, link=BASE_LINK):
    return [CollisionShapeData(*tup) for tup in p.getCollisionShapeData(body, link, physicsClientId=CLIENT)]

def get_data_type(data):
    return data.geometry_type if isinstance(data, CollisionShapeData) else data.visualGeometryType

def get_data_filename(data):
    return data.filename if isinstance(data, CollisionShapeData) else data.meshAssetFileName

def get_default_geometry():
    return {
        'halfExtents': DEFAULT_EXTENTS,
        'radius': DEFAULT_RADIUS,
        'height': DEFAULT_HEIGHT,
        'fileName': DEFAULT_MESH,
        'meshScale': DEFAULT_SCALE,
        'planeNormal': DEFAULT_NORMAL,
    }

DEFAULT_MESH = ''

DEFAULT_EXTENTS = [1, 1, 1]

def get_data_extents(data):
    """
    depends on geometry type:
    for GEOM_BOX: extents,
    for GEOM_SPHERE dimensions[0] = radius,
    for GEOM_CAPSULE and GEOM_CYLINDER, dimensions[0] = height (length), dimensions[1] = radius.
    For GEOM_MESH, dimensions is the scaling factor.
    :return:
    """
    geometry_type = get_data_type(data)
    dimensions = data.dimensions
    if geometry_type == p.GEOM_BOX:
        return dimensions
    return DEFAULT_EXTENTS

DEFAULT_RADIUS = 0.5

def get_data_radius(data):
    geometry_type = get_data_type(data)
    dimensions = data.dimensions
    if geometry_type == p.GEOM_SPHERE:
        return dimensions[0]
    if geometry_type in (p.GEOM_SPHERE, p.GEOM_CAPSULE):
        return dimensions[1]
    return DEFAULT_RADIUS

DEFAULT_HEIGHT = 1

def get_data_height(data):
    geometry_type = get_data_type(data)
    dimensions = data.dimensions
    if geometry_type in (p.GEOM_SPHERE, p.GEOM_CAPSULE):
        return dimensions[0]
    return DEFAULT_HEIGHT

DEFAULT_SCALE = [1, 1, 1]

def get_data_scale(data):
    geometry_type = get_data_type(data)
    dimensions = data.dimensions
    if geometry_type == p.GEOM_MESH:
        return dimensions
    return DEFAULT_SCALE

DEFAULT_NORMAL = [0, 0, 1]

def get_data_normal(data):
    geometry_type = get_data_type(data)
    dimensions = data.dimensions
    if geometry_type == p.GEOM_PLANE:
        return dimensions
    return DEFAULT_NORMAL

def get_data_geometry(data):
    geometry_type = get_data_type(data)
    if geometry_type == p.GEOM_SPHERE:
        parameters = [get_data_radius(data)]
    elif geometry_type == p.GEOM_BOX:
        parameters = [get_data_extents(data)]
    elif geometry_type in (p.GEOM_CYLINDER, p.GEOM_CAPSULE):
        parameters = [get_data_height(data), get_data_radius(data)]
    elif geometry_type == p.GEOM_MESH:
        parameters = [get_data_filename(data), get_data_scale(data)]
    elif geometry_type == p.GEOM_PLANE:
        parameters = [get_data_extents(data)]
    else:
        raise ValueError(geometry_type)
    return SHAPE_TYPES[geometry_type], parameters

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
    # specularColor
    return p.changeVisualShape(body, link, rgbaColor=color, physicsClientId=CLIENT)

#####################################

# Bounding box

def aabb_from_points(points):
    return np.min(points, axis=0), np.max(points, axis=0)

def aabb_union(aabbs):
    return aabb_from_points(np.vstack([aabb for aabb in aabbs]))

def get_lower_upper(body, link=None):
    if link is not None:
        return p.getAABB(body, linkIndex=link, physicsClientId=CLIENT)
    return aabb_union(get_lower_upper(body, link=link)
                      for link in [BASE_LINK] + list(get_links(body)))

get_aabb = get_lower_upper

def get_center_extent(body):
    lower, upper = get_aabb(body)
    center = (np.array(lower) + np.array(upper)) / 2
    extents = (np.array(upper) - np.array(lower))
    return center, extents

def aabb2d_from_aabb(aabb):
    (lower, upper) = aabb
    return lower[:2], upper[:2]

def aabb_contains_aabb(contained, container):
    lower1, upper1 = contained
    lower2, upper2 = container
    return np.greater_equal(lower1, lower2).all() and np.greater_equal(upper2, upper1).all()
    #return np.all(lower2 <= lower1) and np.all(upper1 <= upper2)

def aabb_contains_point(point, container):
    lower, upper = container
    return np.greater_equal(point, lower).all() and np.greater_equal(upper, point).all()
    #return np.all(lower <= point) and np.all(point <= upper)

def get_bodies_in_region(aabb):
    (lower, upper) = aabb
    return p.getOverlappingObjects(lower, upper, physicsClientId=CLIENT)

def approximate_as_prism(body, body_pose=unit_pose()):
    # TODO: make it just orientation
    with PoseSaver(body):
        set_pose(body, body_pose)
        return get_center_extent(body)

def approximate_as_cylinder(body, body_pose=unit_pose()):
    with PoseSaver(body):
        set_pose(body, body_pose)
        center, (width, length, height) = get_center_extent(body)
        diameter = (width + length) / 2 # TODO: check that these are close
        return center, (diameter, height)

#####################################

# Collision

#MAX_DISTANCE = 1e-3
MAX_DISTANCE = 0

def contact_collision():
    step_simulation()
    return len(p.getContactPoints(physicsClientId=CLIENT)) != 0

ContactResult = namedtuple('ContactResult', ['contactFlag', 'bodyUniqueIdA', 'bodyUniqueIdB',
                                         'linkIndexA', 'linkIndexB', 'positionOnA', 'positionOnB',
                                         'contactNormalOnB', 'contactDistance', 'normalForce'])

def pairwise_collision(body1, body2, max_distance=MAX_DISTANCE): # 10000
    # TODO: confirm that this doesn't just check the base link
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                                  physicsClientId=CLIENT)) != 0 # getContactPoints


def pairwise_link_collision(body1, link1, body2, link2, max_distance=MAX_DISTANCE): # 10000
    return len(p.getClosestPoints(bodyA=body1, bodyB=body2, distance=max_distance,
                                  linkIndexA=link1, linkIndexB=link2,
                                  physicsClientId=CLIENT)) != 0 # getContactPoints

#def single_collision(body, max_distance=1e-3):
#    return len(p.getClosestPoints(body, max_distance=max_distance)) != 0

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

RayResult = namedtuple('RayResult', ['objectUniqueId', 'linkIndex',
                                     'hit_fraction', 'hit_position', 'hit_normal'])

def ray_collision(start, end):
    result, = p.rayTest(start, end, physicsClientId=CLIENT)
    return RayResult(*result)

def batch_ray_collision(rays):
    ray_starts = [start for start, _ in rays]
    ray_ends = [end for _, end in rays]
    return [RayResult(*tup) for tup in p.rayTestBatch(ray_starts, ray_ends, physicsClientId=CLIENT)]

#####################################

# Joint motion planning

def get_sample_fn(body, joints, custom_limits={}):
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits, circular_limits=CIRCULAR_LIMITS)
    def fn():
        return tuple(np.random.uniform(lower_limits, upper_limits))
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
    # TODO: use the energy resulting from the mass matrix here?
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
            positions = (1. / (num_steps - i)) * np.array(difference_fn(q2, q)) + q
            q = tuple(positions)
            #q = tuple(wrap_positions(body, joints, positions))
            yield q
    return fn

def refine_path(body, joints, waypoints, num_steps):
    refine_fn = get_refine_fn(body, joints, num_steps)
    refined_path = []
    for v1, v2 in zip(waypoints, waypoints[1:]):
        refined_path += list(refine_fn(v1, v2))
    return refined_path

def get_extend_fn(body, joints, resolutions=None):
    if resolutions is None:
        resolutions = 0.05*np.ones(len(joints))
    difference_fn = get_difference_fn(body, joints)
    def fn(q1, q2):
        steps = np.abs(np.divide(difference_fn(q2, q1), resolutions))
        refine_fn = get_refine_fn(body, joints, num_steps=int(np.max(steps)))
        return refine_fn(q1, q2)
    return fn

def waypoints_from_path(path, tolerance=1e-3):
    if len(path) < 2:
        return path

    def difference_fn(q2, q1):
        return np.array(q2) - np.array(q1)
    #difference_fn = get_difference_fn(body, joints)

    waypoints = [path[0]]
    last_conf = path[1]
    last_difference = get_unit_vector(difference_fn(last_conf, waypoints[-1]))
    for conf in path[2:]:
        difference = get_unit_vector(difference_fn(conf, waypoints[-1]))
        if not np.allclose(last_difference, difference, atol=tolerance, rtol=0):
            waypoints.append(last_conf)
            difference = get_unit_vector(difference_fn(conf, waypoints[-1]))
        last_conf = conf
        last_difference = difference
    waypoints.append(last_conf)
    return waypoints

def get_moving_links(body, moving_joints):
    moving_links = set()
    for link in moving_joints:
        if link not in moving_links:
            moving_links.update(get_link_subtree(body, link))
    return list(moving_links)

def get_moving_pairs(body, moving_joints):
    """
    Check all fixed and moving pairs
    Do not check all fixed and fixed pairs
    Check all moving pairs with a common
    """
    moving_links = get_moving_links(body, moving_joints)
    for link1, link2 in combinations(moving_links, 2):
        ancestors1 = set(get_joint_ancestors(body, link1)) & set(moving_joints)
        ancestors2 = set(get_joint_ancestors(body, link2)) & set(moving_joints)
        if ancestors1 != ancestors2:
            yield link1, link2


def get_self_link_pairs(body, joints, disabled_collisions=set(), only_moving=True):
    moving_links = get_moving_links(body, joints)
    fixed_links = list(set(get_links(body)) - set(moving_links))
    check_link_pairs = list(product(moving_links, fixed_links))
    if only_moving:
        check_link_pairs.extend(get_moving_pairs(body, joints))
    else:
        check_link_pairs.extend(combinations(moving_links, 2))
    check_link_pairs = list(filter(lambda pair: not are_links_adjacent(body, *pair), check_link_pairs))
    check_link_pairs = list(filter(lambda pair: (pair not in disabled_collisions) and
                                                (pair[::-1] not in disabled_collisions), check_link_pairs))
    return check_link_pairs


def get_collision_fn(body, joints, obstacles, attachments, self_collisions, disabled_collisions,
                     custom_limits={}, **kwargs):
    # TODO: convert most of these to keyword arguments
    check_link_pairs = get_self_link_pairs(body, joints, disabled_collisions) if self_collisions else []
    moving_bodies = [body] + [attachment.child for attachment in attachments]
    if obstacles is None:
        obstacles = list(set(get_bodies()) - set(moving_bodies))
    check_body_pairs = list(product(moving_bodies, obstacles))  # + list(combinations(moving_bodies, 2))
    lower_limits, upper_limits = get_custom_limits(body, joints, custom_limits)

    # TODO: maybe prune the link adjacent to the robot
    # TODO: test self collision with the holding
    def collision_fn(q):
        if not all_between(lower_limits, q, upper_limits):
            return True
        set_joint_positions(body, joints, q)
        for attachment in attachments:
            attachment.assign()
        for link1, link2 in check_link_pairs:
            # Self-collisions should not have the max_distance parameter
            if pairwise_link_collision(body, link1, body, link2): #, **kwargs):
                return True
        return any(pairwise_collision(*pair, **kwargs) for pair in check_body_pairs)
    return collision_fn

def plan_waypoints_joint_motion(body, joints, waypoints, start_conf=None, obstacles=None, attachments=[],
                      self_collisions=True, disabled_collisions=set(), custom_limits={}, max_distance=MAX_DISTANCE):
    extend_fn = get_extend_fn(body, joints)
    collision_fn = get_collision_fn(body, joints, obstacles, attachments, self_collisions, disabled_collisions,
                                    custom_limits=custom_limits, max_distance=max_distance)
    if start_conf is None:
        start_conf = get_joint_positions(body, joints)
    else:
        assert len(start_conf) == len(joints)

    for i, waypoint in enumerate([start_conf] + list(waypoints)):
        if collision_fn(waypoint):
            #print("Warning: waypoint configuration {}/{} is in collision".format(i, len(waypoints)))
            return None
    path = [start_conf]
    for waypoint in waypoints:
        assert len(joints) == len(waypoint)
        for q in extend_fn(path[-1], waypoint):
            if collision_fn(q):
                return None
            path.append(q)
    return path

def plan_direct_joint_motion(body, joints, end_conf, **kwargs):
    return plan_waypoints_joint_motion(body, joints, [end_conf], **kwargs)

def check_initial_end(start_conf, end_conf, collision_fn):
    if collision_fn(start_conf):
        print("Warning: initial configuration is in collision")
        return False
    if collision_fn(end_conf):
        print("Warning: end configuration is in collision")
        return False
    return True

def plan_joint_motion(body, joints, end_conf, obstacles=None, attachments=[],
                      self_collisions=True, disabled_collisions=set(),
                      weights=None, resolutions=None, max_distance=MAX_DISTANCE, custom_limits={}, **kwargs):

    assert len(joints) == len(end_conf)
    sample_fn = get_sample_fn(body, joints, custom_limits=custom_limits)
    distance_fn = get_distance_fn(body, joints, weights=weights)
    extend_fn = get_extend_fn(body, joints, resolutions=resolutions)
    collision_fn = get_collision_fn(body, joints, obstacles, attachments, self_collisions, disabled_collisions,
                                    custom_limits=custom_limits, max_distance=max_distance)

    start_conf = get_joint_positions(body, joints)

    if not check_initial_end(start_conf, end_conf, collision_fn):
        return None
    return birrt(start_conf, end_conf, distance_fn, sample_fn, extend_fn, collision_fn, **kwargs)

def plan_lazy_prm(start_conf, end_conf, sample_fn, extend_fn, collision_fn, **kwargs):
    from motion_planners.lazy_prm import lazy_prm
    path, samples, edges, colliding_vertices, colliding_edges = lazy_prm(
        start_conf, end_conf, sample_fn, extend_fn, collision_fn, **kwargs)
    if path is None:
        return path

    #lower, upper = get_custom_limits(body, joints, circular_limits=CIRCULAR_LIMITS)
    def draw_fn(q): # TODO: draw edges instead of vertices
        return np.append(q[:2], [1e-3])
        #return np.array([1, 1, 0.25])*(q + np.array([0., 0., np.pi]))
    handles = []
    for q1, q2 in zip(path, path[1:]):
        handles.append(add_line(draw_fn(q1), draw_fn(q2), color=(0, 1, 0)))
    for i1, i2 in edges:
        color = (0, 0, 1)
        if any(colliding_vertices.get(i, False) for i in (i1, i2)) or colliding_vertices.get((i1, i2), False):
            color = (1, 0, 0)
        elif not colliding_vertices.get((i1, i2), True):
            color = (0, 0, 0)
        handles.append(add_line(draw_fn(samples[i1]), draw_fn(samples[i2]), color=color))
    wait_for_interrupt()
    return path

#####################################

# SE(2) pose motion planning

def plan_base_motion(body, end_conf, base_limits, obstacles=None, direct=False,
                     weights=1*np.ones(3), resolutions=0.05*np.ones(3),
                     max_distance=MAX_DISTANCE, **kwargs):
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
        # TODO: update this function
        set_base_values(body, q)
        if obstacles is None:
            return single_collision(body)
        return any(pairwise_collision(body, obs, max_distance=max_distance) for obs in obstacles)

    start_conf = get_base_values(body)
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

#####################################

# Placements

def stable_z(body, surface, surface_link=None):
    point = get_point(body)
    center, extent = get_center_extent(body)
    _, upper = get_lower_upper(surface, link=surface_link)
    return (upper + extent/2 + (point - center))[2]

def is_placement(body, surface, epsilon=1e-2): # TODO: above / below
    top_aabb = get_lower_upper(body)
    bottom_aabb = get_lower_upper(surface)
    bottom_z_max = bottom_aabb[1][2]
    return (bottom_z_max <= top_aabb[0][2] <= (bottom_z_max + epsilon)) and \
           (aabb_contains_aabb(aabb2d_from_aabb(top_aabb), aabb2d_from_aabb(bottom_aabb)))

def is_center_stable(body, surface, epsilon=1e-2):
    # TODO: compute AABB in origin
    # TODO: use center of mass?
    center, extent = get_center_extent(body)
    base_center = center - np.array([0, 0, extent[2]])/2
    bottom_aabb = get_aabb(surface)
    bottom_z_max = bottom_aabb[1][2]
    #return (bottom_z_max <= base_center[2] <= (bottom_z_max + epsilon)) and \
    return (abs(base_center[2] - bottom_z_max) < epsilon) and \
           (aabb_contains_point(base_center[:2], aabb2d_from_aabb(bottom_aabb)))


def sample_placement(top_body, bottom_body, top_pose=unit_pose(), bottom_link=None,
                     percent=1.0, max_attempts=50, epsilon=1e-3):
    # TODO: transform into the coordinate system of the bottom
    # TODO: maybe I should instead just require that already in correct frame
    bottom_aabb = get_lower_upper(bottom_body, link=bottom_link)
    for _ in range(max_attempts):
        theta = np.random.uniform(*CIRCULAR_LIMITS)
        rotation = Euler(yaw=theta)
        set_pose(top_body, multiply(Pose(euler=rotation), top_pose))
        center, extent = get_center_extent(top_body)
        lower = (np.array(bottom_aabb[0]) + percent*extent/2)[:2]
        upper = (np.array(bottom_aabb[1]) - percent*extent/2)[:2]
        if np.greater(lower, upper).any():
            continue
        x, y = np.random.uniform(lower, upper)
        z = (bottom_aabb[1] + extent/2.)[2] + epsilon
        point = np.array([x, y, z]) + (get_point(top_body) - center)
        pose = multiply(Pose(point, rotation), top_pose)
        set_pose(top_body, pose)
        return pose
    return None

#####################################

# Reachability

def sample_reachable_base(robot, point, reachable_range=(0.25, 1.0), max_attempts=50):
    for _ in range(max_attempts):
        radius = np.random.uniform(*reachable_range)
        x, y = radius*unit_from_theta(np.random.uniform(-np.pi, np.pi)) + point[:2]
        yaw = np.random.uniform(*CIRCULAR_LIMITS)
        base_values = (x, y, yaw)
        #set_base_values(robot, base_values)
        return base_values
    return None

def uniform_pose_generator(robot, gripper_pose, **kwargs):
    point = point_from_pose(gripper_pose)
    while True:
        base_values = sample_reachable_base(robot, point)
        if base_values is None:
            break
        yield base_values
        #set_base_values(robot, base_values)
        #yield get_pose(robot)

#####################################

# Constraints - applies forces when not satisfied

def get_constraints():
    """
    getConstraintUniqueId will take a serial index in range 0..getNumConstraints,  and reports the constraint unique id.
    Note that the constraint unique ids may not be contiguous, since you may remove constraints.
    """
    return [p.getConstraintUniqueId(i, physicsClientId=CLIENT)
            for i in range(p.getNumConstraints(physicsClientId=CLIENT))]

def remove_constraint(constraint):
    p.removeConstraint(constraint, physicsClientId=CLIENT)

ConstraintInfo = namedtuple('ConstraintInfo', ['parentBodyUniqueId', 'parentJointIndex',
                                               'childBodyUniqueId', 'childLinkIndex', 'constraintType',
                                               'jointAxis', 'jointPivotInParent', 'jointPivotInChild',
                                               'jointFrameOrientationParent', 'jointFrameOrientationChild', 'maxAppliedForce'])

def get_constraint_info(constraint): # getConstraintState
    # TODO: four additional arguments
    return ConstraintInfo(*p.getConstraintInfo(constraint, physicsClientId=CLIENT)[:11])

def get_fixed_constraints():
    fixed_constraints = []
    for constraint in get_constraints():
        constraint_info = get_constraint_info(constraint)
        if constraint_info.constraintType == p.JOINT_FIXED:
            fixed_constraints.append(constraint)
    return fixed_constraints

def add_fixed_constraint(body, robot, robot_link, max_force=None):
    body_link = BASE_LINK
    body_pose = get_pose(body)
    #body_pose = get_com_pose(body, link=body_link)
    #end_effector_pose = get_link_pose(robot, robot_link)
    end_effector_pose = get_com_pose(robot, robot_link)
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
    constraint = p.createConstraint(robot, robot_link, body, body_link,  # Both seem to work
                                    p.JOINT_FIXED, jointAxis=unit_point(),
                                    parentFramePosition=point,
                                    childFramePosition=unit_point(),
                                    parentFrameOrientation=quat,
                                    childFrameOrientation=unit_quat(),
                                    physicsClientId=CLIENT)
    if max_force is not None:
        p.changeConstraint(constraint, maxForce=max_force, physicsClientId=CLIENT)
    return constraint

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

def create_attachment(parent, parent_link, child):
    parent_link_pose = get_link_pose(parent, parent_link)
    child_pose = get_pose(child)
    grasp_pose = multiply(invert(parent_link_pose), child_pose)
    return Attachment(parent, parent_link, grasp_pose, child)

def body_from_end_effector(end_effector_pose, grasp_pose):
    """
    world_from_parent * parent_from_child = world_from_child
    """
    return multiply(end_effector_pose, grasp_pose)

def end_effector_from_body(body_pose, grasp_pose):
    """
    grasp_pose: the body's pose in gripper's frame

    world_from_child * (parent_from_child)^(-1) = world_from_parent
    (parent: gripper, child: body to be grasped)

    Pose_{world,gripper} = Pose_{world,block}*Pose_{block,gripper}
                         = Pose_{world,block}*(Pose_{gripper,block})^{-1}
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
    return p.setJointMotorControl2(bodyUniqueId=body,
                                   jointIndex=joint,
                                   controlMode=p.POSITION_CONTROL,
                                   targetPosition=value,
                                   targetVelocity=0.0,
                                   maxVelocity=get_max_velocity(body, joint),
                                   force=get_max_force(body, joint),
                                   physicsClientId=CLIENT)

def control_joints(body, joints, positions):
    # TODO: the whole PR2 seems to jitter
    #kp = 1.0
    #kv = 0.3
    #forces = [get_max_force(body, joint) for joint in joints]
    #forces = [5000]*len(joints)
    #forces = [20000]*len(joints)
    return p.setJointMotorControlArray(body, joints, p.POSITION_CONTROL,
                                       targetPositions=positions,
                                       targetVelocities=[0.0] * len(joints),
                                       physicsClientId=CLIENT) #,
                                        #positionGains=[kp] * len(joints),
                                        #velocityGains=[kv] * len(joints),)
                                        #forces=forces)

def joint_controller(body, joints, target, tolerance=1e-3):
    assert(len(joints) == len(target))
    positions = get_joint_positions(body, joints)
    while not np.allclose(positions, target, atol=tolerance, rtol=0):
        control_joints(body, joints, target)
        yield positions
        positions = get_joint_positions(body, joints)

def joint_controller_hold(body, joints, target, **kwargs):
    """
    Keeps other joints in place
    """
    movable_joints = get_movable_joints(body)
    conf = list(get_joint_positions(body, movable_joints))
    for joint, value in zip(movable_from_joints(body, joints), target):
        conf[joint] = value
    return joint_controller(body, movable_joints, conf, **kwargs)

def trajectory_controller(body, joints, path, **kwargs):
    for target in path:
        for positions in joint_controller(body, joints, target, **kwargs):
            yield positions

def simulate_controller(controller, max_time=np.inf): # Allow option to sleep rather than yield?
    sim_dt = get_time_step()
    sim_time = 0.0
    for _ in controller:
        if max_time < sim_time:
            break
        step_simulation()
        sim_time += sim_dt
        yield sim_time

def velocity_control_joints(body, joints, velocities):
    #kv = 0.3
    return p.setJointMotorControlArray(body, joints, p.VELOCITY_CONTROL,
                                       targetVelocities=velocities,
                                       physicsClientId=CLIENT) #,
                                        #velocityGains=[kv] * len(joints),)
                                        #forces=forces)

#####################################

def compute_jacobian(robot, link, positions): #, joints=None):
    #if joints is None:
    joints = get_movable_joints(robot)
    assert len(joints) == len(positions)
    velocities = [0.0] * len(positions)
    accelerations = [0.0] * len(positions)
    translate, rotate = p.calculateJacobian(robot, link, unit_point(), positions,
                                            velocities, accelerations, physicsClientId=CLIENT)
    #movable_from_joints(robot, joints)
    return zip(*translate), zip(*rotate) # len(joints) x 3


def compute_joint_weights(robot, num=100):
    # http://openrave.org/docs/0.6.6/_modules/openravepy/databases/linkstatistics/#LinkStatisticsModel
    start_time = time.time()
    joints = get_movable_joints(robot)
    sample_fn = get_sample_fn(robot, joints)
    weighted_jacobian = np.zeros(len(joints))
    links = list(get_links(robot))
    # links = {l for j in joints for l in get_link_descendants(self.robot, j)}
    masses = [get_mass(robot, link) for link in links]  # Volume, AABB volume
    total_mass = sum(masses)
    for _ in range(num):
        conf = sample_fn()
        for mass, link in zip(masses, links):
            translate, rotate = compute_jacobian(robot, link, conf)
            weighted_jacobian += np.array([mass * np.linalg.norm(vec) for vec in translate]) / total_mass
    weighted_jacobian /= num
    print(list(weighted_jacobian))
    print(time.time() - start_time)
    return weighted_jacobian

#####################################

def inverse_kinematics_helper(robot, link, target_pose):
    (target_point, target_quat) = target_pose
    assert target_point is not None
    if target_quat is None:
        kinematic_conf = p.calculateInverseKinematics(robot, link, target_point, physicsClientId=CLIENT)
    else:
        kinematic_conf = p.calculateInverseKinematics(robot, link, target_point, target_quat, physicsClientId=CLIENT)
    if (kinematic_conf is None) or any(map(math.isnan, kinematic_conf)):
        return None
    return kinematic_conf

def is_pose_close(pose, target_pose, pos_tolerance=1e-3, ori_tolerance=1e-3*np.pi):
    (point, quat) = pose
    (target_point, target_quat) = target_pose
    if (target_point is not None) and not np.allclose(point, target_point, atol=pos_tolerance, rtol=0):
        return False
    if (target_quat is not None) and not np.allclose(quat, target_quat, atol=ori_tolerance, rtol=0):
        # TODO: account for quaternion redundancy
        return False
    return True

def inverse_kinematics(robot, link, target_pose, max_iterations=200, custom_limits={}, **kwargs):
    movable_joints = get_movable_joints(robot)
    for iterations in range(max_iterations):
        # TODO: stop is no progress
        # TODO: stop if collision or invalid joint limits
        kinematic_conf = inverse_kinematics_helper(robot, link, target_pose)
        if kinematic_conf is None:
            return None
        set_joint_positions(robot, movable_joints, kinematic_conf)
        if is_pose_close(get_link_pose(robot, link), target_pose, **kwargs):
            break
    else:
        return None
    lower_limits, upper_limits = get_custom_limits(robot, movable_joints, custom_limits)
    if not all_between(lower_limits, kinematic_conf, upper_limits):
        return None
    return kinematic_conf

#####################################

def get_cartesian_waypoints(start_point, direction, quat, step_size=0.01):
    distance = get_length(direction)
    unit_direction = get_unit_vector(direction)
    for t in np.arange(0, distance, step_size):
        point = start_point + t*unit_direction
        yield (point, quat)
    yield (start_point + direction, quat)

def get_quaternion_waypoints(point, start_quat, end_quat, step_size=np.pi/16):
    angle = quat_angle_between(start_quat, end_quat)
    for t in np.arange(0, angle, step_size):
        quat = quaternion_slerp(start_quat, end_quat, fraction=t/angle)
        yield (point, quat)
    yield (point, end_quat)

def interpolate_poses(pose1, pose2, pos_step_size=0.01, ori_step_size=np.pi/16):
    pos1, quat1 = pose1
    pos2, quat2 = pose2
    num_steps = int(math.ceil(max(get_distance(pos1, pos2)/pos_step_size,
                                  quat_angle_between(quat1, quat2)/ori_step_size)))
    for i in range(num_steps):
        fraction = float(i) / num_steps
        pos = (1-fraction)*np.array(pos1) + fraction*np.array(pos2)
        quat = quaternion_slerp(quat1, quat2, fraction=fraction)
        yield (pos, quat)
    yield pose2

def workspace_trajectory(robot, link, start_point, direction, quat, **kwargs):
    # TODO: pushing example
    # TODO: just use current configuration?
    # TODO: check collisions?
    # TODO: lower intermediate tolerance
    traj = []
    for pose in get_cartesian_waypoints(start_point, direction, quat):
        conf = inverse_kinematics(robot, link, pose, **kwargs)
        if conf is None:
            return None
        traj.append(conf)
    return traj

#####################################

def plan_cartesian_motion(robot, first_joint, target_link, waypoint_poses,
                          max_iterations=200, custom_limits={}, **kwargs):
    # TODO: fix stationary joints
    # TODO: pass in set of movable joints and take least common ancestor
    # TODO: update with most recent bullet updates
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics.py
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics_husky_kuka.py
    # TODO: plan a path without needing to following intermediate waypoints

    lower_limits, upper_limits = get_custom_limits(robot, get_movable_joints(robot), custom_limits)
    selected_links = get_link_subtree(robot, first_joint)
    selected_movable_joints = prune_fixed_joints(robot, selected_links)
    assert(target_link in selected_links)
    selected_target_link = selected_links.index(target_link)
    sub_robot = clone_body(robot, links=selected_links, visual=False, collision=False) # TODO: joint limits
    sub_movable_joints = get_movable_joints(sub_robot)

    solutions = []
    for target_pose in waypoint_poses:
        for _ in range(max_iterations):
            sub_kinematic_conf = inverse_kinematics_helper(sub_robot, selected_target_link, target_pose)
            if sub_kinematic_conf is None:
                remove_body(sub_robot)
                return None
            set_joint_positions(sub_robot, sub_movable_joints, sub_kinematic_conf)
            if is_pose_close(get_link_pose(sub_robot, selected_target_link), target_pose, **kwargs):
                set_joint_positions(robot, selected_movable_joints, sub_kinematic_conf)
                kinematic_conf = get_configuration(robot)
                if not all_between(lower_limits, kinematic_conf, upper_limits):
                    remove_body(sub_robot)
                    return None
                solutions.append(kinematic_conf)
                break
        else:
            remove_body(sub_robot)
            return None
    remove_body(sub_robot)
    return solutions

def sub_inverse_kinematics(robot, first_joint, target_link, target_pose, **kwargs):
    solutions = plan_cartesian_motion(robot, first_joint, target_link, [target_pose], **kwargs)
    if solutions:
        return solutions[0]
    return None

#####################################

def get_lifetime(lifetime):
    if lifetime is None:
        return 0
    return lifetime

def add_text(text, position=(0, 0, 0), color=(0, 0, 0), lifetime=None, parent=-1, parent_link=BASE_LINK):
    return p.addUserDebugText(str(text), textPosition=position, textColorRGB=color, # textSize=1,
                              lifeTime=get_lifetime(lifetime), parentObjectUniqueId=parent, parentLinkIndex=parent_link,
                              physicsClientId=CLIENT)

def add_line(start, end, color=(0, 0, 0), width=1, lifetime=None, parent=-1, parent_link=BASE_LINK):
    return p.addUserDebugLine(start, end, lineColorRGB=color, lineWidth=width,
                              lifeTime=get_lifetime(lifetime), parentObjectUniqueId=parent, parentLinkIndex=parent_link,
                              physicsClientId=CLIENT)

def remove_debug(debug): # removeAllUserDebugItems
    p.removeUserDebugItem(debug, physicsClientId=CLIENT)

def add_body_name(body, name=None, **kwargs):
    if name is None:
        name = get_name(body)
    with PoseSaver(body):
        set_pose(body, unit_pose())
        lower, upper = get_aabb(body)
    #position = (0, 0, upper[2])
    position = upper
    return add_text(name, position=position, parent=body, **kwargs)  # removeUserDebugItem


def add_segments(points, closed=False, **kwargs):
    lines = []
    for v1, v2 in zip(points, points[1:]):
        lines.append(add_line(v1, v2, **kwargs))
    if closed:
        lines.append(add_line(points[-1], points[0], **kwargs))
    return lines

def draw_pose(pose, length=0.1, **kwargs):
    origin_world = tform_point(pose, np.zeros(3))
    handles = []
    for k in range(3):
        axis = np.zeros(3)
        axis[k] = 1
        axis_world = tform_point(pose, length*axis)
        handles.append(add_line(origin_world, axis_world, color=axis, **kwargs))
    return handles

def draw_base_limits(limits, z=1e-2, **kwargs):
    lower, upper = limits
    vertices = [(lower[0], lower[1], z), (lower[0], upper[1], z),
                (upper[0], upper[1], z), (upper[0], lower[1], z)]
    return add_segments(vertices, closed=True, **kwargs)

def draw_aabb(aabb, **kwargs):
    lower, upper = aabb
    d = len(lower)
    vertices = list(product(range(len(aabb)), repeat=d))
    lines = []
    for i1, i2 in combinations(vertices, 2):
        if sum(i1[k] != i2[k] for k in range(d)) == 1:
            p1 = [aabb[i1[k]][k] for k in range(d)]
            p2 = [aabb[i2[k]][k] for k in range(d)]
            lines.append(add_line(p1, p2, **kwargs))
    return lines

def draw_point(point, size=0.01, **kwargs):
    lines = []
    for i in range(len(point)):
        axis = np.zeros(len(point))
        axis[i] = 1.0
        p1 = np.array(point) - size/2 * axis
        p2 = np.array(point) + size/2 * axis
        lines.append(add_line(p1, p2, **kwargs))
    return lines
    #extent = size * np.ones(len(point)) / 2
    #aabb = np.array(point) - extent, np.array(point) + extent
    #return draw_aabb(aabb, **kwargs)

def draw_mesh(mesh, **kwargs):
    verts, faces = mesh
    lines = []
    for indices in faces:
        #for i1, i2 in combinations(indices, 2):
        for i1, i2 in zip(indices[-1] +  indices[-1:], indices):
            lines.append(add_line(verts[i1], verts[i2], **kwargs))
    return lines

#####################################

# Polygonal surfaces

def create_rectangular_surface(width, length):
    extents = np.array([width, length, 0]) / 2.
    unit_corners = [(-1, -1), (+1, -1), (+1, +1), (-1, +1)]
    return [np.append(c, 0) * extents for c in unit_corners]

def is_point_in_polygon(point, polygon):
    sign = None
    for i in range(len(polygon)):
        v1, v2 = np.array(polygon[i - 1][:2]), np.array(polygon[i][:2])
        delta = v2 - v1
        normal = np.array([-delta[1], delta[0]])
        dist = normal.dot(point[:2] - v1)
        if i == 0:  # TODO: equality?
            sign = np.sign(dist)
        elif np.sign(dist) != sign:
            return False
    return True

def tform_point(affine, point):
    return point_from_pose(multiply(affine, Pose(point=point)))

def apply_affine(affine, points):
    return [tform_point(affine, p) for p in points]

def is_mesh_on_surface(polygon, world_from_surface, mesh, world_from_mesh, epsilon=1e-2):
    surface_from_mesh = multiply(invert(world_from_surface), world_from_mesh)
    points_surface = apply_affine(surface_from_mesh, mesh.vertices)
    min_z = np.min(points_surface[:, 2])
    return (abs(min_z) < epsilon) and \
           all(is_point_in_polygon(p, polygon) for p in points_surface)

def is_point_on_surface(polygon, world_from_surface, point_world):
    [point_surface] = apply_affine(invert(world_from_surface), [point_world])
    return is_point_in_polygon(point_surface, polygon[::-1])

def sample_polygon_tform(polygon, points):
    min_z = np.min(points[:, 2])
    aabb_min = np.min(polygon, axis=0)
    aabb_max = np.max(polygon, axis=0)
    while True:
        x = np.random.uniform(aabb_min[0], aabb_max[0])
        y = np.random.uniform(aabb_min[1], aabb_max[1])
        theta = np.random.uniform(0, 2 * np.pi)
        point = Point(x, y, -min_z)
        quat = Euler(yaw=theta)
        surface_from_origin = Pose(point, quat)
        yield surface_from_origin
        # if all(is_point_in_polygon(p, polygon) for p in apply_affine(surface_from_origin, points)):
        #  yield surface_from_origin

def sample_surface_pose(polygon, world_from_surface, mesh):
    for surface_from_origin in sample_polygon_tform(polygon, mesh.vertices):
        world_from_mesh = multiply(world_from_surface, surface_from_origin)
        if is_mesh_on_surface(polygon, world_from_surface, mesh, world_from_mesh):
            yield world_from_mesh

#####################################

# Sampling edges

def sample_categorical(categories):
    from bisect import bisect
    names = categories.keys()
    cutoffs = np.cumsum([categories[name] for name in names])/sum(categories.values())
    return names[bisect(cutoffs, np.random.random())]

def sample_edge_point(polygon, radius):
    edges = zip(polygon, polygon[-1:] + polygon[:-1])
    edge_weights = {i: max(get_length(v2 - v1) - 2 * radius, 0) for i, (v1, v2) in enumerate(edges)}
    # TODO: fail if no options
    while True:
        index = sample_categorical(edge_weights)
        v1, v2 = edges[index]
        t = np.random.uniform(radius, get_length(v2 - v1) - 2 * radius)
        yield t * get_unit_vector(v2 - v1) + v1

def get_closest_edge_point(polygon, point):
    # TODO: always pick perpendicular to the edge
    edges = zip(polygon, polygon[-1:] + polygon[:-1])
    best = None
    for v1, v2 in edges:
        proj = (v2 - v1)[:2].dot((point - v1)[:2])
        if proj <= 0:
            closest = v1
        elif get_length((v2 - v1)[:2]) <= proj:
            closest = v2
        else:
            closest = proj * get_unit_vector((v2 - v1))
        if (best is None) or (get_length((point - closest)[:2]) < get_length((point - best)[:2])):
            best = closest
    return best

def sample_edge_pose(polygon, world_from_surface, mesh):
    radius = max(get_length(v[:2]) for v in mesh.vertices)
    origin_from_base = Pose(Point(z=p.min(mesh.vertices[:, 2])))
    for point in sample_edge_point(polygon, radius):
        theta = np.random.uniform(0, 2 * np.pi)
        surface_from_origin = Pose(point, Euler(yaw=theta))
        yield multiply(world_from_surface, surface_from_origin, origin_from_base)

#####################################

# Convex Hulls

def convex_hull(points):
    # TODO: 2D convex hull
    from scipy.spatial import ConvexHull
    # TODO: cKDTree is faster, but KDTree can do all pairs closest
    hull = ConvexHull(points)
    new_indices = {i: ni for ni, i in enumerate(hull.vertices)}
    vertices = hull.points[hull.vertices, :]
    faces = np.vectorize(lambda i: new_indices[i])(hull.simplices)
    return vertices, faces

def mesh_from_points(points):
    vertices, indices = convex_hull(points)
    new_indices = []
    for triplet in indices:
        centroid = np.average(vertices[triplet], axis=0)
        v1, v2, v3 = vertices[triplet]
        normal = np.cross(v3 - v1, v2 - v1)
        if normal.dot(centroid) > 0:
            # if normal.dot(centroid) < 0:
            triplet = triplet[::-1]
        new_indices.append(tuple(triplet))
    return vertices.tolist(), new_indices

def mesh_from_body(body, link=BASE_LINK):
    # TODO: read obj files so I can always obtain the pointcloud
    # TODO: approximate cylindrical/spherical using convex hull
    # TODO: change based on geom_type
    print(get_collision_data(body, link))
    print(get_visual_data(body, link))
    # TODO: these aren't working...

#####################################

# Mesh & Pointcloud Files

def obj_file_from_mesh(mesh):
    """
    Creates a *.obj mesh string
    :param mesh: tuple of list of vertices and list of faces
    :return: *.obj mesh string
    """
    vertices, faces = mesh
    s = 'g Mesh\n' # TODO: string writer
    for v in vertices:
        assert(len(v) == 3)
        s += '\nv {}'.format(' '.join(map(str, v)))
    for f in faces:
        assert(len(f) == 3)
        f = [i+1 for i in f]
        s += '\nf {}'.format(' '.join(map(str, f)))
        s += '\nf {}'.format(' '.join(map(str, reversed(f))))
    return s

def read_mesh_off(path, scale=1.0):
    """
    Reads a *.off mesh file
    :param path: path to the *.off mesh file
    :return: tuple of list of vertices and list of faces
    """
    with open(path) as f:
        assert (f.readline().split()[0] == 'OFF'), 'Not OFF file'
        nv, nf, ne = [int(x) for x in f.readline().split()]
        verts = [tuple(scale * float(v) for v in f.readline().split()) for _ in range(nv)]
        faces = [tuple(map(int, f.readline().split()[1:])) for _ in range(nf)]
        return verts, faces


def read_pcd_file(path):
    """
    Reads a *.pcd pointcloud file
    :param path: path to the *.pcd pointcloud file
    :return: list of points
    """
    with open(path) as f:
        data = f.readline().split()
        num_points = 0
        while data[0] != 'DATA':
            if data[0] == 'POINTS':
                num_points = int(data[1])
            data = f.readline().split()
            continue
        return [tuple(map(float, f.readline().split())) for _ in range(num_points)]

# TODO: factor out things that don't depend on pybullet

#####################################

"""
def readWrl(filename, name='wrlObj', scale=1.0, color='black'):
    def readOneObj():
        vl = []
        while True:
            line = fl.readline()
            split = line.split(',')
            if len(split) != 2:
                break
            split = split[0].split()
            if len(split) == 3:
                vl.append(np.array([scale*float(x) for x in split]+[1.0]))
            else:
                break
        print '    verts', len(vl),
        verts = np.vstack(vl).T
        while line.split()[0] != 'coordIndex':
            line = fl.readline()
        line = fl.readline()
        faces = []
        while True:
            line = fl.readline()
            split = line.split(',')
            if len(split) > 3:
                faces.append(np.array([int(x) for x in split[:3]]))
            else:
                break
        print 'faces', len(faces)
        return Prim(verts, faces, hu.Pose(0,0,0,0), None,
                    name=name+str(len(prims)))

    fl = open(filename)
    assert fl.readline().split()[0] == '#VRML', 'Not VRML file?'
    prims = []
    while True:
        line = fl.readline()
        if not line: break
        split = line.split()
        if not split or split[0] != 'point':
            continue
        else:
            print 'Object', len(prims)
            prims.append(readOneObj())
    # Have one "part" so that shadows are simpler
    part = Shape(prims, None, name=name+'_part')
    # Keep color only in top entry.
    return Shape([part], None, name=name, color=color)
"""
