import math
import os
import random
import re
import numpy as np

from collections import namedtuple
from pr2_never_collisions import NEVER_COLLISIONS
from utils import multiply, get_link_pose, joint_from_name, set_joint_position, set_joint_positions, \
    get_joint_positions, get_min_limit, get_max_limit, quat_from_euler, read_pickle, set_pose, set_base_values, \
    get_pose, euler_from_quat, link_from_name, has_link, point_from_pose, invert, Pose, unit_point, unit_quat, \
    unit_pose, get_center_extent, joints_from_names, PoseSaver, get_lower_upper, get_joint_limits, get_joints, \
    ConfSaver, get_bodies, create_mesh, remove_body, get_body_name, single_collision, unit_from_theta

PR2_URDF = "models/pr2_description/pr2.urdf"
DRAKE_PR2_URDF = "models/drake/pr2_description/urdf/pr2_simplified.urdf"

TOP_HOLDING_LEFT_ARM = [0.67717021, -0.34313199, 1.2, -1.46688405, 1.24223229, -1.95442826, 2.22254125]
SIDE_HOLDING_LEFT_ARM = [0.39277395, 0.33330058, 0., -1.52238431, 2.72170996, -1.21946936, -2.98914779]
REST_LEFT_ARM = [2.13539289, 1.29629967, 3.74999698, -0.15000005, 10000., -0.10000004, 10000.]
WIDE_LEFT_ARM = [1.5806603449288885, -0.14239066980481405, 1.4484623937179126, -1.4851759349218694, 1.3911839347271555,
                 -1.6531320011389408, -2.978586584568441]
CENTER_LEFT_ARM = [-0.07133691252641006, -0.052973836083405494, 1.5741805775919033, -1.4481146328076862,
                   1.571782540186805, -1.4891468812835686, -9.413338322697955]
# WIDE_RIGHT_ARM = [-1.3175723551150083, -0.09536552225976803, -1.396727055561703, -1.4433371993320296, -1.5334243909312468, -1.7298129320065025, 6.230244924007009]

PR2_LEFT_ARM_CONFS = {
    'top': TOP_HOLDING_LEFT_ARM,
}

PR2_GROUPS = {
    'base': ['x', 'y', 'theta'],
    'torso': ['torso_lift_joint'],
    'head': ['head_pan_joint', 'head_tilt_joint'],
    'left_arm': ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint',
                 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint'],
    'right_arm': ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint',
                  'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint'],
    'left_gripper': ['l_gripper_l_finger_joint', 'l_gripper_r_finger_joint',
                     'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_tip_joint'],
    'right_gripper': ['r_gripper_l_finger_joint', 'r_gripper_r_finger_joint',
                      'r_gripper_l_finger_tip_joint', 'r_gripper_r_finger_tip_joint'],
}
PR2_TOOL_FRAMES = {
    'left_gripper': 'l_gripper_palm_link',  # l_gripper_palm_link | l_gripper_tool_frame
    'right_gripper': 'r_gripper_palm_link',  # r_gripper_palm_link | r_gripper_tool_frame
}

ARM_LINK_NAMES = {
    'left': 'l_gripper_palm_link',  # l_gripper_tool_frame,
    'right': 'r_gripper_palm_link',
}

HEAD_LINK_NAME = 'high_def_optical_frame' # high_def_optical_frame | high_def_frame | wide_stereo_l_stereo_camera_frame | ...
# 'head_mount_kinect_rgb_optical_frame'
# 'head_mount_kinect_rgb_link'

TORSO_JOINT_NAME = 'torso_lift_joint'
# LEFT_TOOL_NAME = 'l_gripper_tool_frame' # l_gripper_tool_joint | l_gripper_tool_frame

TOOL_POSE = ([0.18, 0., 0.], [0., 0.70710678, 0., 0.70710678])
TOOL_DIRECTION = [0., 0., 1.]

#####################################

PR2_JOINT_NAMES = []

def set_pr2_joint_names(pr2):
    for joint in get_joints(pr2):
        PR2_JOINT_NAMES.append(joint)

def get_pr2_joints(joint_names):
    joint_from_name = dict(zip(PR2_JOINT_NAMES, range(len(PR2_JOINT_NAMES))))
    return [joint_from_name[name] for name in joint_names]

#####################################

def rightarm_from_leftarm(config):
    # right_from_left = np.array([-1, 1, -1, 1, -1, 1, 1])
    right_from_left = np.array([-1, 1, -1, 1, -1, 1, -1])  # Drake
    return config * right_from_left

def arm_conf(arm, config):
    if arm == 'left':
        return config
    elif arm == 'right':
        return rightarm_from_leftarm(config)
    else:
        raise ValueError(arm)


def get_carry_conf(arm, grasp_type):
    if grasp_type == 'top':
        return arm_conf(arm, TOP_HOLDING_LEFT_ARM)
    elif grasp_type == 'side':
        return arm_conf(arm, SIDE_HOLDING_LEFT_ARM)
    else:
        raise NotImplementedError()

ARM_NAMES = ('left', 'right')

def get_other_arm(arm):
    for other_arm in ARM_NAMES:
        if other_arm != arm:
            return other_arm
    raise ValueError(arm)


def get_disabled_collisions(pr2):
    #disabled_names = PR2_ADJACENT_LINKS
    #disabled_names = PR2_DISABLED_COLLISIONS
    disabled_names = NEVER_COLLISIONS
    #disabled_names = PR2_DISABLED_COLLISIONS + NEVER_COLLISIONS
    return [(link_from_name(pr2, name1), link_from_name(pr2, name2))
            for name1, name2 in disabled_names if has_link(pr2, name1) and has_link(pr2, name2)]


def load_dae_collisions():
    # pr2-beta-static.dae: link 0 = base_footprint
    # pybullet: link -1 = base_footprint
    dae_file = 'models/pr2_description/pr2-beta-static.dae'
    dae_string = open(dae_file).read()
    link_regex = r'<\s*link\s+sid="(\w+)"\s+name="(\w+)"\s*>'
    link_mapping = dict(re.findall(link_regex, dae_string))
    ignore_regex = r'<\s*ignore_link_pair\s+link0="kmodel1/(\w+)"\s+link1="kmodel1/(\w+)"\s*/>'
    disabled_collisions = []
    for link1, link2 in re.findall(ignore_regex, dae_string):
        disabled_collisions.append((link_mapping[link1], link_mapping[link2]))
    return disabled_collisions


def load_srdf_collisions():
    srdf_file = 'models/pr2_description/pr2.srdf'
    srdf_string = open(srdf_file).read()
    regex = r'<\s*disable_collisions\s+link1="(\w+)"\s+link2="(\w+)"\s+reason="(\w+)"\s*/>'
    disabled_collisions = []
    for link1, link2, reason in re.findall(regex, srdf_string):
        if reason == 'Never':
            disabled_collisions.append((link1, link2))
    return disabled_collisions


#####################################

# End-effectors

def get_arm_group(arm):
    assert (arm in ARM_NAMES)
    return '{}_arm'.format(arm)


def get_arm_joints(robot, arm):
    return joints_from_names(robot, PR2_GROUPS[get_arm_group(arm)])


def get_arm_conf(robot, arm):
    return get_joint_positions(robot, get_arm_joints(robot, arm))


def set_arm_conf(robot, arm, conf):
    set_joint_positions(robot, get_arm_joints(robot, arm), conf)


def get_gripper_link(robot, arm):
    assert arm in ARM_LINK_NAMES
    return link_from_name(robot, ARM_LINK_NAMES[arm])


# def get_gripper_pose(robot):
#    # world_from_gripper * gripper_from_tool * tool_from_object = world_from_object
#    pose = multiply(get_link_pose(robot, link_from_name(robot, LEFT_ARM_LINK)), TOOL_POSE)
#    #pose = get_link_pose(robot, link_from_name(robot, LEFT_TOOL_NAME))
#    return pose

def get_group_conf(robot, group):
    return get_joint_positions(robot, joints_from_names(robot, PR2_GROUPS[group]))

def set_group_conf(robot, group, positions):
    set_joint_positions(robot, joints_from_names(robot, PR2_GROUPS[group]), positions)

def open_arm(robot, arm):  # TODO: these are mirrored on the pr2
    gripper_name = '{}_gripper'.format(arm)
    for name in PR2_GROUPS[gripper_name]:
        joint = joint_from_name(robot, name)
        set_joint_position(robot, joint, get_max_limit(robot, joint))


def close_arm(robot, arm):
    gripper_name = '{}_gripper'.format(arm)
    for name in PR2_GROUPS[gripper_name]:
        joint = joint_from_name(robot, name)
        set_joint_position(robot, joint, get_min_limit(robot, joint))

#####################################

# Grasps

#GRASP_LENGTH = 0.04
GRASP_LENGTH = 0.
#GRASP_LENGTH = -0.01
MAX_GRASP_WIDTH = 0.07


def get_top_grasps(body, under=False, tool_pose=TOOL_POSE,
                   max_width=MAX_GRASP_WIDTH, grasp_length=GRASP_LENGTH):
    pose = get_pose(body)
    set_pose(body, unit_pose())
    center, (w, l, h) = get_center_extent(body)
    reflect_z = (np.zeros(3), quat_from_euler([0, math.pi, 0]))
    translate = ([0, 0, h / 2 - grasp_length], unit_quat())
    grasps = []
    if w <= max_width:
        for i in range(1 + under):
            rotate_z = (unit_point(), quat_from_euler([0, 0, math.pi / 2 + i * math.pi]))
            grasps += [multiply(tool_pose, translate, rotate_z, reflect_z)]
    if l <= max_width:
        for i in range(1 + under):
            rotate_z = (unit_point(), quat_from_euler([0, 0, i * math.pi]))
            grasps += [multiply(tool_pose, translate, rotate_z, reflect_z)]
    set_pose(body, pose)
    return grasps


def get_side_grasps(body, under=False, limits=True, grasp_length=GRASP_LENGTH):
    pose = get_pose(body)
    set_pose(body, unit_pose())
    center, (w, l, h) = get_center_extent(body)
    grasps = []
    for j in range(1 + under):
        swap_xz = (unit_point(), quat_from_euler([0, -math.pi / 2 + j * math.pi, 0]))
        # if not limits or (w <= MAX_GRASP_WIDTH):
        if True:
            translate = ([0, 0, l / 2 - grasp_length], unit_quat())
            for i in range(2):
                rotate_z = (unit_point(), quat_from_euler([math.pi / 2 + i * math.pi, 0, 0]))
                grasps += [multiply(TOOL_POSE, translate, rotate_z, swap_xz)]  # , np.array([w])
        if True:
            # if not limits or (l <= MAX_GRASP_WIDTH):
            translate = ([0, 0, w / 2 - grasp_length], unit_quat())
            for i in range(2):
                rotate_z = (unit_point(), quat_from_euler([i * math.pi, 0, 0]))
                grasps += [multiply(TOOL_POSE, translate, rotate_z, swap_xz)]  # , np.array([l])
    set_pose(body, pose)
    return grasps


def get_x_presses(body, max_orientations=1):  # g_f_o
    pose = get_pose(body)
    set_pose(body, unit_pose())
    center, (w, l, h) = get_center_extent(body)
    press_poses = []
    for j in range(max_orientations):
        swap_xz = (unit_point(), quat_from_euler([0, -math.pi / 2 + j * math.pi, 0]))
        translate = ([0, 0, w / 2], unit_quat())
        press_poses += [multiply(TOOL_POSE, translate, swap_xz)]
    set_pose(body, pose)
    return press_poses


GET_GRASPS = {
    'top': get_top_grasps,
    'side': get_side_grasps,
    # 'press': get_x_presses,
}

#####################################

# Inverse reachability

DATABASES_DIR = 'databases'
IR_FILENAME = '{}_{}_ir.pickle'

def get_database_file(filename):
    directory = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(directory, DATABASES_DIR, filename)


def load_inverse_reachability(arm, grasp_type):
    filename = IR_FILENAME.format(grasp_type, arm)
    path = get_database_file(filename)
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

#####################################

# Camera

WIDTH, HEIGHT = 640, 480
FX, FY = 772.55, 772.5
MAX_VISUAL_DISTANCE = 5.0
MAX_KINECT_DISTANCE = 2.5

def get_camera_matrix(width, height, fx, fy):
    # cx, cy = 320.5, 240.5
    cx, cy = width / 2., height / 2.
    return np.array([
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]])

def ray_from_pixel(camera_matrix, pixel):
    return np.linalg.inv(camera_matrix).dot(np.append(pixel, 1))

def pixel_from_ray(camera_matrix, ray):
    return camera_matrix.dot(ray / ray[2])[:2]

def get_pr2_camera_matrix():
    return get_camera_matrix(WIDTH, HEIGHT, FX, FY)

def get_pr2_view_section(z):
    camera_matrix = get_pr2_camera_matrix()
    pixels = [(0, 0), (WIDTH, HEIGHT)]
    return [z*ray_from_pixel(camera_matrix, p) for p in pixels]

def is_visible_point(camera_matrix, depth, point):
    if not (0 <= point[2] < depth):
        return False
    px, py = pixel_from_ray(camera_matrix, point)
    return (0 <= px < WIDTH) and (0 <= py < HEIGHT)

def is_visible_aabb(body_lower, body_upper):
    z = body_lower[2]
    if z < 0:
        return False
    view_lower, view_upper = get_pr2_view_section(z)
    return not (np.any(body_lower[:2] < view_lower[:2]) or
                np.any(view_upper[:2] < body_upper[:2]))

def support_from_aabb(lower, upper):
    min_x, min_y, z = lower
    max_x, max_y, _ = upper
    return [(min_x, min_y, z), (min_x, max_y, z),
            (max_x, max_y, z), (max_x, min_y, z)]

def cone_mesh_from_support(support):
    assert(len(support) == 4)
    vertices = [np.zeros(3)] + support
    faces = [(1, 4, 3), (1, 3, 2)]
    for i in range(len(support)):
        index1 = 1+i
        index2 = 1+(i+1)%len(support)
        faces.append((0, index1, index2))
    return vertices, faces

def get_detection_cone(pr2, body, depth=MAX_VISUAL_DISTANCE):
    head_link = link_from_name(pr2, HEAD_LINK_NAME)
    with PoseSaver(body):
        body_head = multiply(invert(get_link_pose(pr2, head_link)), get_pose(body))
        set_pose(body, body_head)
        body_lower, body_upper = get_lower_upper(body)
        z = body_lower[2]
        if depth < z:
            return None, z
        if not is_visible_aabb(body_lower, body_upper):
            return None, z
        return cone_mesh_from_support(support_from_aabb(body_lower, body_upper)), z

def get_cone_mesh(depth=MAX_VISUAL_DISTANCE):
    # TODO: attach to the pr2?
    cone = [(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT)]
    camera_matrix = get_pr2_camera_matrix()
    vertices = []
    for pixel in cone:
        ray = depth * ray_from_pixel(camera_matrix, pixel)
        vertices.append(ray[:3])
    return cone_mesh_from_support(vertices)

def plan_scan_path(pr2, tilt=0):
    head_joints = joints_from_names(pr2, PR2_GROUPS['head'])
    start_conf = get_joint_positions(pr2, head_joints)
    lower_limit, upper_limit = get_joint_limits(pr2, head_joints)

    first_conf = np.array([lower_limit[0], tilt])
    second_conf = np.array([upper_limit[0], tilt])
    if start_conf[0] > 0:
        first_conf, second_conf = second_conf, first_conf
    third_conf = np.array([0, tilt])
    return [start_conf, first_conf, second_conf, third_conf]

def inverse_visibility(pr2, point, head_name=HEAD_LINK_NAME):
    # TODO: test visibility by getting box
    # TODO: IK version
    # https://github.com/PR2/pr2_controllers/blob/kinetic-devel/pr2_head_action/src/pr2_point_frame.cpp
    #head_joints = [joint_from_name(pr2, name) for name in PR2_GROUPS['head']]
    #head_link = head_joints[-1]
    optical_frame = ('optical' in head_name)
    head_link = link_from_name(pr2, head_name)

    head_joints = joints_from_names(pr2, PR2_GROUPS['head'])
    with ConfSaver(pr2):
        set_joint_positions(pr2, head_joints, np.zeros(len(head_joints)))
        head_pose = get_link_pose(pr2, head_link)
    point_head = point_from_pose(multiply(invert(head_pose), Pose(point)))
    if optical_frame:
        dy, dz, dx =  np.array([-1, -1, 1])*np.array(point_head)
    else:
        dx, dy, dz = np.array(point_head)
    theta = np.math.atan2(dy, dx)  # TODO: might need to negate the minus for the default one
    phi = np.math.atan2(-dz, np.sqrt(dx ** 2 + dy ** 2))
    conf = [theta, phi]
    # TODO: test joint limits
    #pose = Pose(point_from_pose(head_pose), Euler(pitch=phi, yaw=theta)) # TODO: initial roll?
    return conf

#####################################

Detection = namedtuple('Detection', ['body', 'distance'])

def get_detections(pr2, p_false_neg=0, **kwargs):
    detections = []
    for body in get_bodies():
        if np.random.random() < p_false_neg:
            continue
        mesh, z = get_detection_cone(pr2, body, **kwargs)
        if mesh is None:
            continue
        cone = create_mesh(mesh, color=None)
        if not single_collision(cone):
            detections.append(Detection(body, z))
        remove_body(cone)
    return detections

def get_visual_detections(pr2, **kwargs):
    return [body for body, _ in get_detections(pr2, depth=MAX_VISUAL_DISTANCE, **kwargs)]

def get_kinect_registrations(pr2, **kwargs):
    return [body for body, _ in get_detections(pr2, depth=MAX_KINECT_DISTANCE, **kwargs)]

# TODO: Gaussian on resulting pose

#####################################

# TODO: base motion with some stochasticity
def visible_base_generator(robot, target_point, base_range):
    #base_from_table = point_from_pose(get_pose(robot))[:2]
    while True:
        base_from_table = unit_from_theta(np.random.uniform(0, 2 * np.pi))
        look_distance = np.random.uniform(*base_range)
        base_xy = target_point[:2] - look_distance * base_from_table
        base_theta = np.math.atan2(base_from_table[1], base_from_table[0]) # TODO: stochastic orientation?
        base_q = np.append(base_xy, base_theta)
        yield base_q