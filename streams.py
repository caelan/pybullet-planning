from pybullet_utils import get_joint_type, is_movable, get_joint_limits, create_box, invert, multiply, \
    get_max_velocity, get_num_joints, get_movable_joints, get_joint_name, get_name, get_point, get_base_values, \
    set_base_values, set_pose, get_link_pose, joint_from_name, link_from_name, set_joint_position, get_joint_position, \
    get_body_names, get_joint_names, pairwise_collision, get_colliding_links, self_collision, env_collision, \
    set_joint_positions, get_joint_positions, sample_placement, sample_reachable_base, add_data_path, connect, \
    filtered_self_collision, get_safe_colliding_links, get_pose, write_pickle, read_pickle, point_from_pose
from pr2_utils import TOP_HOLDING_LEFT_ARM, LEFT_ARM_LINK, LEFT_JOINT_NAMES, RIGHT_JOINT_NAMES, TOOL_POSE, TORSO_JOINT, \
    TOP_HOLDING_RIGHT_ARM, get_top_grasps, REST_RIGHT_ARM, \
    inverse_kinematics, inverse_kinematics_helper, learned_pose_generator, uniform_pose_generator

class Pose(object):
    #def __init__(self, position, orientation):
    #    self.position = position
    #    self.orientation = orientation
    def __init__(self, body, value):
        self.body = body
        self.value = value
    def __repr__(self):
        return 'p{}'.format(id(self) % 1000)

class Grasp(object):
    def __init__(self, body, value):
        self.body = body
        self.value = value
    def __repr__(self):
        return 'g{}'.format(id(self) % 1000)

class Conf(object):
    def __init__(self, body, joints, values):
        self.body = body
        self.joints = joints
        self.values = values
    def __repr__(self):
        return 'q{}'.format(id(self) % 1000)

class Trajectory(object):
    def __init__(self, path):
        self.path = path
    def __repr__(self):
        return 't{}'.format(id(self) % 1000)

def get_motion_gen(problem):
    def gen(bq1, bq2):
        path = [bq1, bq2]
        bt = Trajectory(path)
        return (bt,)
    return gen

def get_grasp_gen(problem):
    def gen(body):
        grasps = []
        if 'top' in problem.grasp_types:
            for grasp in get_top_grasps(body):
                g = Grasp(body, grasp)
                grasps += [(g,)]
        if 'side' in problem.grasp_types:
            raise NotImplementedError()
        return grasps
    return gen

def get_stable_gen(problem):
    def gen(body, surface):
        while True:
            body_pose = sample_placement(body, surface)
            if body_pose is None:
                break
            p = Pose(body, body_pose)
            # TODO: collisions with fixed bodies
            yield [(p,)]
    return gen

def get_ik_ir_gen(problem):
    pr2 = problem.robot

    def gen(a, o, p, g):
        gripper_pose = multiply(p.value, invert(g.value))
        link = link_from_name(pr2, LEFT_ARM_LINK) if a == 'left' else link_from_name(pr2, None)

        default_conf = TOP_HOLDING_LEFT_ARM
        left_joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]


        base_generator = learned_pose_generator(pr2, gripper_pose)
        #base_generator = uniform_pose_generator(pr2, gripper_pose)
        while True:
            set_joint_positions(pr2, left_joints, default_conf)
            set_pose(pr2, next(base_generator))
            #if pairwise_collision(pr2, table):
            #    continue

            conf = inverse_kinematics_helper(pr2, link, gripper_pose)
            if (conf is None): # or pairwise_collision(pr2, table):
                continue
            bp = Pose(pr2, get_pose(pr2))
            mt = Trajectory([])
            yield (bp, mt)
    return gen