import pybullet as p
import time
import pybullet_data
import argparse
import numpy as np
from motion_planners.rrt_connect import birrt, direct_path


REST_LEFT_ARM = [2.13539289, 1.29629967, 3.74999698, -0.15000005, 10000., -0.10000004, 10000.]
TOP_HOLDING_LEFT_ARM = [0.67717021, -0.34313199, 1.2, -1.46688405, 1.24223229, -1.95442826, 2.22254125]
SIDE_HOLDING_LEFT_ARM = [0.39277395, 0.33330058, 0., -1.52238431, 2.72170996, -1.21946936, -2.98914779]
REST_LEFT_ARM = [2.13539289, 1.29629967, 3.74999698, -0.15000005, 10000., -0.10000004, 10000.]
WIDE_LEFT_ARM = [1.5806603449288885, -0.14239066980481405, 1.4484623937179126, -1.4851759349218694, 1.3911839347271555, -1.6531320011389408, -2.978586584568441]
CENTER_LEFT_ARM = [-0.07133691252641006, -0.052973836083405494, 1.5741805775919033, -1.4481146328076862, 1.571782540186805, -1.4891468812835686, -9.413338322697955]
WIDE_RIGHT_ARM = [-1.3175723551150083, -0.09536552225976803, -1.396727055561703, -1.4433371993320296, -1.5334243909312468, -1.7298129320065025, 6.230244924007009]

LEFT_ARM_LINK = 'l_gripper_tool_frame' # l_gripper_palm_link | l_gripper_tool_frame

LEFT_JOINT_NAMES = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
                    'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
RIGHT_JOINT_NAMES = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                     'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
HEAD_JOINT_NAMES = ['head_pan_joint', 'head_tilt_joint']

LEFT_GRIPPER_NAME = 'l_gripper_l_finger_joint'
LEFT_TOOL_NAME = 'l_gripper_tool_frame' # l_gripper_tool_joint | l_gripper_tool_frame


TOOL_TFORM = [[0., 0., 1., 0.18],
              [0., 1., 0., 0.],
              [-1., 0., 0., 0.],
              [0., 0., 0., 1.]]

TOOL_POSE = ([0.18, 0., 0.], [0., 0.70710678, 0., 0.70710678])
TOOL_DIRECTION = [ 0., 0., 1.]

# https://github.com/ros/geometry/blob/hydro-devel/tf/src/tf/transformations.py

def rightarm_from_leftarm(config):
  right_from_left = np.array([-1, 1, -1, 1, -1, 1, 1])
  return config*right_from_left

REST_RIGHT_ARM = rightarm_from_leftarm(REST_LEFT_ARM)

#LEFT_ARM_JOINTS = [15,16,17,18,19,20,21]
#RIGHT_ARM_JOINTS = [27,28,29,30,31,32,33]
#HEAD_JOINTS = [13,14]
# openrave-robot.py robots/pr2-beta-static.zae --info manipulators

TORSO_JOINT = 'torso_lift_joint'

REVOLUTE_LIMITS = -np.pi, np.pi
#REVOLUTE_LIMITS = -10000, 10000

BASE_LIMITS = ([-2.5, -2.5, 0], [2.5, 2.5, 0])

class Pose(object):
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation

class Conf(object):
    def __init__(self):
        pass

# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#

def get_joint_type(body, joint):
    return p.getJointInfo(body, joint)[2]

def is_movable(body, joint):
    return get_joint_type(body, joint) != p.JOINT_FIXED

def is_circular(body, joint):
    lower = p.getJointInfo(body, joint)[8]
    upper = p.getJointInfo(body, joint)[9]
    return upper < lower

def get_joint_limits(body, joint):
    if is_circular(body, joint):
        return REVOLUTE_LIMITS
    return p.getJointInfo(body, joint)[8:10]

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

def invert((point, quat)):
    return p.invertTransform(point, quat)

def multiply((point1, quat1), (point2, quat2)):
    return p.multiplyTransforms(point1, quat1, point2, quat2) # TODO: variable number of args

def create_plane():
    collision_id = p.createVisualShape(p.GEOM_PLANE, normal=[])

def create_mesh():
    raise NotImplementedError()

def create_cylinder(radius, height):
    collision_id =  p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)

def create_capsule(radius, height):
    collision_id = p.createCollisionShape(p.GEOM_CAPSULE, radius=radius, height=height)

def create_sphere(radius):
    collision_id = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)

def get_lower_upper(body):
    return p.getAABB(body)

def get_center_extent(body):
    lower, upper = get_lower_upper(body)
    center = (np.array(lower) + upper) / 2
    extents = (np.array(upper) - lower)
    return center, extents

def get_shape_data(body):
    return p.getVisualShapeData(body)

def get_max_velocity(body, joint):
    return p.getJointInfo(body, joint)[11]

def get_num_joints(body):
    return p.getNumJoints(body)

def get_joints(body):
    return range(get_num_joints(body))

def get_movable_joints(body): # 45 / 87 on pr2
    return [joint for joint in get_joints(body) if is_movable(body, joint)]

def joint_from_movable(body, index):
    return get_joints(body)[index]

def get_joint_name(body, joint):
    return p.getJointInfo(body, joint)[1]

def get_link_name(body, link):
    return p.getJointInfo(body, link)[12]

def get_name(body):
    return p.getBodyInfo(body)[1]

def get_base_link(body):
    return p.getBodyInfo(body)[0]

def get_pose(body):
    return p.getBasePositionAndOrientation(body)
    #point, quat = p.getBasePositionAndOrientation(body) # [x,y,z,w]
    #return np.concatenate([point, quat])

def get_point(body):
    return p.getBasePositionAndOrientation(body)[0]

def get_quat(body):
    return p.getBasePositionAndOrientation(body)[1]

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

def set_pose(body, point, quat):
    p.resetBasePositionAndOrientation(body, point, quat)

def set_point(body, point):
    _, quat = p.getBasePositionAndOrientation(body)
    p.resetBasePositionAndOrientation(body, point, quat)

def set_quat(body, quat):
    p.resetBasePositionAndOrientation(body, get_point(body), quat)

def get_link_pose(body, link): # Local vs world?
    #point, quat = p.getLinkState(body, link)[0:2] # Local
    #point, quat = p.getLinkState(body, link)[2:4] # World
    point, quat = p.getLinkState(body, link)[4:6] # World
    return point, quat
    #return np.concatenate([point, quat])

def joint_from_name(body, name):
    for joint in xrange(get_num_joints(body)):
        if get_joint_name(body, joint) == name:
            return joint
    raise ValueError(body, name)

def link_from_name(body, name):
    for link in xrange(get_num_joints(body)):
        if get_link_name(body, link) == name:
            return link
    raise ValueError(body, name)

def body_from_name(name):
    for body in xrange(get_num_bodies()):
        if get_name(body) == name:
            return body
    raise ValueError(name)

#def set_joint(body, joint, value):
#    p.setJointMotorControl2(bodyUniqueId=body,
#                            jointIndex=joint,
#                            controlMode=p.POSITION_CONTROL,
#                            targetPosition=value,
#                            force=maxForce)

def set_joint_position(body, joint, value):
    p.resetJointState(body, joint, value)

def get_joint_position(body, joint):
    return p.getJointState(body, joint)[0]

def get_num_bodies():
    return p.getNumBodies()

def get_bodies():
    return range(get_num_bodies())

def get_body_names():
    return map(get_name, get_bodies())

def get_joints(body):
    return range(get_num_joints(body))

def get_joint_names(body):
    return [get_joint_name(body, joint) for joint in get_joints(body)]

def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler)

def euler_from_quat(quat):
    return p.getEulerFromQuaternion(quat)

def z_rotation(theta):
    return quat_from_euler([0, 0, theta])

def matrix_from_quat(quat):
    return p.getMatrixFromQuaternion(quat)

#def quat_from_matrix(matrix):
#    return p.getQuaternionFromMatrix(matrix)

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

def pairwise_collision(body1, body2, max_distance=0.001): # 10000
    return len(p.getClosestPoints(body1, body2, max_distance)) != 0 # getContactPoints

def env_collision(body1):
    for body2 in get_bodies():
        if (body1 != body2) and pairwise_collision(body1, body2):
        #if pairwise_collision(body1, body2):
            return True
    return False

def wrap_angle(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi

def wrap_joint(body, joint, value):
    if is_circular(body, joint):
        return wrap_angle(value)
    return value

def circular_difference(theta2, theta1):
    return wrap_angle(theta2 - theta1)

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

def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        set_joint_position(body, joint, value)

def get_joint_positions(body, joints):
    return tuple(get_joint_position(body, joint) for joint in joints)

def violates_limits(body, joints, values):
    for joint, value in zip(joints, values):
        if not is_circular(body, joint):
            lower, upper = get_joint_limits(body, joint)
            if (value < lower) or (upper < value):
                return True
    return False

def sample_joints(body, joints):
    values = []
    for joint in joints:
        limits = REVOLUTE_LIMITS if is_circular(body, joint) \
            else get_joint_limits(body, joint)
        values.append(np.random.uniform(*limits))
    return tuple(values)

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

def is_connected():
    return p.getConnectionInfo()['isConnected']

def get_connection():
    return p.getConnectionInfo()['connectionMethod']

def has_gui():
    return get_connection() == p.GUI

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

def unit_from_theta(theta):
    return np.array([np.cos(theta), np.sin(theta)])

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

GRASP_LENGTH = 0.04
MAX_GRASP_WIDTH = 0.07
import math

def get_top_grasps(body, under=False, limits=True, grasp_length=GRASP_LENGTH):
    #w, l, h = np.max(mesh.vertices, axis=0) - \
    #          np.min(mesh.vertices, axis=0)
    h = 0.15
    reflect_z = (np.zeros(3), quat_from_euler([0, math.pi, 0]))
    translate = ([0, 0, h / 2 - grasp_length], quat_from_euler(np.zeros(3)))
    #if not limits or (w <= MAX_GRASP_WIDTH):
    for i in range(1 + under):
        rotate_z = (np.zeros(3), quat_from_euler([0, 0, math.pi / 2 + i * math.pi]))
        yield multiply(multiply(translate, rotate_z), reflect_z)
    #if not limits or (l <= MAX_GRASP_WIDTH):
    for i in range(1 + under):
        rotate_z = (np.zeros(3), quat_from_euler([0, 0, i * math.pi]))
        yield multiply(multiply(translate, rotate_z), reflect_z)

# def get_side_grasps(mesh, under=False, limits=True, grasp_length=GRASP_LENGTH):
#   w, l, h = np.max(mesh.vertices, axis=0) - \
#             np.min(mesh.vertices, axis=0)
#   for j in range(1 + under):
#     swap_xz = trans_from_quat(quat_from_angle_vector(-math.pi/2 + j*math.pi, [0, 1, 0]))
#     if not limits or (w <= MAX_GRASP_WIDTH):
#       translate = trans_from_point(0, 0, l / 2 - grasp_length)
#       for i in range(2):
#         rotate_z = trans_from_quat(quat_from_angle_vector(math.pi / 2 + i * math.pi, [1, 0, 0]))
#         yield translate.dot(rotate_z).dot(swap_xz), np.array([w])
#     if not limits or (l <= MAX_GRASP_WIDTH):
#       translate = trans_from_point(0, 0, w / 2 - grasp_length)
#       for i in range(2):
#         rotate_z = trans_from_quat(quat_from_angle_vector(i * math.pi, [1, 0, 0]))
#         yield translate.dot(rotate_z).dot(swap_xz), np.array([l])

def inverse_kinematics(robot, target_pose):
    #print get_base_link(robot)
    #base_pose = get_link_pose(robot, link_from_name(robot, get_base_link(robot)))
    #pose = multiply(invert(get_pose(robot)), pose)
    (point, quat) = target_pose
    link = link_from_name(robot, LEFT_ARM_LINK)

    movable_joints = get_movable_joints(robot)
    current_conf = get_joint_positions(robot, movable_joints)
    # TODO: constrain not moving
    left_joints = [joint_from_name(robot, name) for name in LEFT_JOINT_NAMES]

    #min_limits = [get_joint_limits(robot, joint)[0] for joint in movable_joints]
    #max_limits = [get_joint_limits(robot, joint)[1] for joint in movable_joints]
    min_limits = [get_joint_limits(robot, joint)[0] for joint in get_joints(robot)]
    max_limits = [get_joint_limits(robot, joint)[1] for joint in get_joints(robot)]
    print min_limits
    print max_limits
    print left_joints
    print [joint_from_name(robot, name) for name in RIGHT_JOINT_NAMES]
    print movable_joints
    #min_limits = [get_joint_limits(robot, joint)[0] if joint in left_joints else current_conf[i]
    #              for i, joint in enumerate(movable_joints)]
    #max_limits = [get_joint_limits(robot, joint)[1] if joint in left_joints else current_conf[i]
    #              for i, joint in enumerate(movable_joints)]
    max_velocities = [get_max_velocity(robot, joint) for joint in get_joints(robot)] # Range of Jacobian
    #max_velocities = [get_max_velocity(robot, joint) for joint in movable_joints] # Range of Jacobian
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
    damping = tuple(0.1*np.ones(get_num_joints(robot)))
    print damping
    min_limits = 'fis'
    max_limits = 'stuff' # TODO: these are not used?
    max_velocities = 'what'
    current_conf = 'aefawfe'

    # TODO: fix particular joints?
    t0 = time.time()
    max_iterations = 150
    kinematic_conf = current_conf
    for iterations in xrange(max_iterations): # 0.000863273143768 / iteration
        #kinematic_conf = p.calculateInverseKinematics(robot, link, point, quat)
        #kinematic_conf = p.calculateInverseKinematics(robot, link, point)
        kinematic_conf = p.calculateInverseKinematics(robot, link, point,
                                                      #quat,
                                                      lowerLimits=min_limits, upperLimits=max_limits,
                                                      jointRanges=max_velocities, restPoses=current_conf,
                                                      #jointDamping=damping,
                                              )
        #kinematic_conf = p.calculateInverseKinematics(robot, link, point, quat,
        #                                              min_limits, max_limits,
        #                                              max_velocities, current_conf)
        set_joint_positions(robot, movable_joints, kinematic_conf)
        link_point, link_quat = get_link_pose(robot, link)
        print link_point, link_quat
        if np.allclose(link_point, point, atol=1e-3) and np.allclose(link_quat, quat, atol=1e-3):
            print iterations
            break
        print link_point, link_quat
    else:
        #return None
        return kinematic_conf
    total_time = (time.time() - t0)
    print total_time
    print (time.time() - t0)/max_iterations
    #kinematic_conf = p.calculateInverseKinematics(robot, link, point)
    return kinematic_conf

def get_gripper_pose(robot):
    # world_from_gripper * gripper_from_tool * tool_from_object = world_from_object
    pose = multiply(get_link_pose(robot, link_from_name(robot, LEFT_ARM_LINK)), TOOL_POSE)
    #pose = get_link_pose(robot, link_from_name(robot, LEFT_TOOL_NAME))
    return pose

def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    client = p.connect(p.GUI) if args.viewer else p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    print pybullet_data.getDataPath()

    #p.setGravity(0, 0, -10)
    #planeId = p.loadURDF("plane.urdf")
    #table = p.loadURDF("table/table.urdf", 0, 0, 0, 0, 0, 0.707107, 0.707107)

    # boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
    # boxId = p.loadURDF("pr2.urdf")
    pr2 = p.loadURDF("/Users/caelan/Programs/Installation/pr2_description/pr2_local.urdf",
                     useFixedBase=False) # flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT
    #pr2 = p.loadURDF("pr2_description/urdf/pr2_simplified.urdf", useFixedBase=False)
    origin = (0, 0, 0)

    print pr2
    # for i in range (10000):
    #    p.stepSimulation()
    #    time.sleep(1./240.)

    print get_joint_names(pr2)
    print joint_from_name(pr2, TORSO_JOINT)
    print get_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT))

    raw_input('Continue?')

    print set_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT), 0.2)  # Updates automatically

    left_joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]
    right_joints = [joint_from_name(pr2, name) for name in RIGHT_JOINT_NAMES]
    print set_joint_positions(pr2, left_joints, TOP_HOLDING_LEFT_ARM) # TOP_HOLDING_LEFT_ARM | SIDE_HOLDING_LEFT_ARM
    print set_joint_positions(pr2, right_joints, REST_RIGHT_ARM)

    print get_name(pr2)
    print get_body_names()
    # print p.getBodyUniqueId(pr2)
    print get_joint_names(pr2)

    #for joint, value in zip(LEFT_ARM_JOINTS, REST_LEFT_ARM):
    #    set_joint_position(pr2, joint, value)
    # for name, value in zip(LEFT_JOINT_NAMES, REST_LEFT_ARM):
    #     joint = joint_from_name(pr2, name)
    #     #print name, joint, get_joint_position(pr2, joint), value
    #     print name, get_joint_limits(pr2, joint), get_joint_type(pr2, joint), get_link_name(pr2, joint)
    #     set_joint_position(pr2, joint, value)
    #     #print name, joint, get_joint_position(pr2, joint), value
    # for name, value in zip(RIGHT_JOINT_NAMES, REST_RIGHT_ARM):
    #     set_joint_position(pr2, joint_from_name(pr2, name), value)

    print p.getNumJoints(pr2)
    jointId = 0
    print p.getJointInfo(pr2, jointId)
    print p.getJointState(pr2, jointId)

    # for i in xrange(10):
    #     #lower, upper = BASE_LIMITS
    #     #q = np.random.rand(len(lower))*(np.array(upper) - np.array(lower)) + lower
    #     q = np.random.uniform(*BASE_LIMITS)
    #     theta = np.random.uniform(*REVOLUTE_LIMITS)
    #     quat = z_rotation(theta)
    #     print q, theta, quat, env_collision(pr2)
    #     #set_point(pr2, q)
    #     set_pose(pr2, q, quat)
    #     #p.getMouseEvents()
    #     #p.getKeyboardEvents()
    #     raw_input('Continue?') # Stalls because waiting for input
    #
    # # TODO: self collisions
    # for i in xrange(10):
    #     for name in LEFT_JOINT_NAMES:
    #         joint = joint_from_name(pr2, name)
    #         value = np.random.uniform(*get_joint_limits(pr2, joint))
    #         set_joint_position(pr2, joint, value)
    #     raw_input('Continue?')



    #start = (-2, -2, 0)
    #set_base_values(pr2, start)
    # #start = get_base_values(pr2)
    # goal = (2, 2, 0)
    # p.addUserDebugLine(start, goal, lineColorRGB=(1, 1, 0)) # addUserDebugText
    # print start, goal
    # raw_input('Plan?')
    # path = plan_base_motion(pr2, goal)
    # print path
    # if path is None:
    #     return
    # print len(path)
    # for bq in path:
    #     set_base_values(pr2, bq)
    #     raw_input('Continue?')



    # left_joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]
    # for joint in left_joints:
    #     print joint, get_joint_name(pr2, joint), get_joint_limits(pr2, joint), \
    #         is_circular(pr2, joint), get_joint_position(pr2, joint)
    #
    # #goal = np.zeros(len(left_joints))
    # goal = []
    # for name, value in zip(LEFT_JOINT_NAMES, REST_LEFT_ARM):
    #     joint = joint_from_name(pr2, name)
    #     goal.append(wrap_joint(pr2, joint, value))
    #
    # path = plan_joint_motion(pr2, left_joints, goal)
    # print path
    # for q in path:s
    #     set_joint_positions(pr2, left_joints, q)
    #     raw_input('Continue?')

    print p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, p.JOINT_FIXED, p.JOINT_POINT2POINT, p.JOINT_GEAR # 0 1 4 5 6

    print len(get_movable_joints(pr2))
    for joint in xrange(get_num_joints(pr2)):
        if is_movable(pr2, joint):
            print joint, get_joint_name(pr2, joint), get_joint_type(pr2, joint)

    #joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]
    #set_joint_positions(pr2, joints, sample_joints(pr2, joints))
    #print get_joint_positions(pr2, joints) # Need to print before the display updates?

    set_base_values(pr2, (1, -1, -np.pi/4))
    movable_joints = get_movable_joints(pr2)
    gripper_pose = get_link_pose(pr2, link_from_name(pr2, LEFT_ARM_LINK))
    print gripper_pose
    print get_joint_positions(pr2, movable_joints)
    p.addUserDebugLine(origin, gripper_pose[0], lineColorRGB=(1, 0, 0))
    p.stepSimulation()
    raw_input('Pre2 IK')
    set_joint_positions(pr2, left_joints, SIDE_HOLDING_LEFT_ARM) # TOP_HOLDING_LEFT_ARM | SIDE_HOLDING_LEFT_ARM
    print get_joint_positions(pr2, movable_joints)
    p.stepSimulation()
    raw_input('Pre IK')
    conf = inverse_kinematics(pr2, gripper_pose) # Doesn't automatically set configuraitons
    print conf
    print get_joint_positions(pr2, movable_joints)
    set_joint_positions(pr2, movable_joints, conf)
    print get_link_pose(pr2, link_from_name(pr2, LEFT_ARM_LINK))
    #print get_joint_positions(pr2, movable_joints)
    p.stepSimulation()
    raw_input('Post IK')
    return

    #box = create_box(.07, .05, .15)
    # print pose_from_tform(TOOL_TFORM)
    # gripper_pose = get_link_pose(pr2, link_from_name(pr2, LEFT_ARM_LINK))
    # #gripper_pose = multiply(gripper_pose, TOOL_POSE)
    # #gripper_pose = get_gripper_pose(pr2)
    # for i, grasp_pose in enumerate(get_top_grasps(box)):
    #     grasp_pose = multiply(TOOL_POSE, grasp_pose)
    #     box_pose = multiply(gripper_pose, grasp_pose)
    #     set_pose(box, *box_pose)
    #     print get_pose(box)
    #     raw_input('Grasp {}'.format(i))
    # return

    # default_conf = get_joint_positions(pr2, movable_joints)
    # for _ in xrange(100):
    #     #box_pose = sample_placement(box, table)
    #     box_pose = ((0, 0, 1), quat_from_euler(np.zeros(3)))
    #     set_pose(box, *box_pose)
    #     base_values = sample_reachable_base(pr2, get_point(box))
    #     #for grasp_pose in list(get_top_grasps(box))[:1]:
    #     for grasp_pose in get_top_grasps(box):
    #         grasp_pose = multiply(TOOL_POSE, grasp_pose)
    #         gripper_pose = multiply(box_pose, invert(grasp_pose))
    #         p.addUserDebugLine(origin, gripper_pose[0], lineColorRGB=(1, 1, 0))
    #         set_joint_positions(pr2, movable_joints, default_conf)
    #         set_base_values(pr2, base_values)
    #         conf = inverse_kinematics(pr2, gripper_pose)
    #         print gripper_pose
    #         print conf
    #         print get_base_values(pr2)
    #         p.stepSimulation()
    #         raw_input('IK Solution')


    link = link_from_name(pr2, LEFT_ARM_LINK)
    point, quat = get_link_pose(pr2, link)
    print point, quat
    p.addUserDebugLine(origin, point, lineColorRGB=(1, 1, 0))  # addUserDebugText
    raw_input('Continue?')

    current_conf = get_joint_positions(pr2, movable_joints)

    #ik_conf = p.calculateInverseKinematics(pr2, link, point)
    #ik_conf = p.calculateInverseKinematics(pr2, link, point, quat)

    min_limits = [get_joint_limits(pr2, joint)[0] for joint in movable_joints]
    max_limits = [get_joint_limits(pr2, joint)[1] for joint in movable_joints]
    max_velocities = [get_max_velocity(pr2, joint) for joint in movable_joints] # Range of Jacobian
    print min_limits
    print max_limits
    print max_velocities
    ik_conf = p.calculateInverseKinematics(pr2, link, point, quat, lowerLimits=min_limits,
                                           upperLimits=max_limits, jointRanges=max_velocities, restPoses=current_conf)


    value_from_joint = dict(zip(movable_joints, ik_conf))
    print [value_from_joint[joint] for joint in joints]

    #print len(ik_conf), ik_conf
    set_joint_positions(pr2, movable_joints, ik_conf)
    #print len(movable_joints), get_joint_positions(pr2, movable_joints)
    print get_joint_positions(pr2, joints)

    raw_input('Finish?')

    p.disconnect()

    # createConstraint


if __name__ == '__main__':
    main()