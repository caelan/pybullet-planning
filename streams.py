from pybullet_utils import invert, multiply, get_body_name, set_pose, get_link_pose, joint_from_name, link_from_name, set_joint_position, get_joint_position, \
    pairwise_collision, set_joint_positions, get_joint_positions, sample_placement, get_pose, \
    unit_quat
from pr2_utils import TOP_HOLDING_LEFT_ARM, SIDE_HOLDING_LEFT_ARM, LEFT_ARM_LINK, LEFT_JOINT_NAMES, \
    get_top_grasps, get_side_grasps, close_arm, open_arm, arm_conf, \
    inverse_kinematics, inverse_kinematics_helper, learned_pose_generator, TOOL_DIRECTION, ARM_LINK_NAMES
from problems import get_fixed_bodies

import pybullet as p
import numpy as np

class Pose(object):
    #def __init__(self, position, orientation):
    #    self.position = position
    #    self.orientation = orientation
    def __init__(self, body, value):
        self.body = body
        self.value = tuple(value)
    def step(self):
        set_pose(self.body, self.value)
    def __repr__(self):
        return 'p{}'.format(id(self) % 1000)

class Grasp(object):
    def __init__(self, body, value, approach, carry):
        self.body = body
        self.value = tuple(value) # gripper_from_object
        self.approach = tuple(approach)
        self.carry = tuple(carry)
    def __repr__(self):
        return 'g{}'.format(id(self) % 1000)

class Conf(object):
    def __init__(self, body, joints, values):
        self.body = body
        self.joints = joints
        self.values = tuple(values)
    def step(self):
        set_joint_positions(self.body, self.joints, self.values)
    def __repr__(self):
        return 'q{}'.format(id(self) % 1000)

class Trajectory(object):
    def __init__(self, path):
        self.path = tuple(path)
        # TODO: constructor that takes in this info
    #def step(self):
    #    #for q in self.path:
    #    for q in self.path[1:]:
    #    #for q in self.path[1::5]:
    #        q.step()
    #        #set_joint_position(q.body, q.joints, q.values)
    #        #p.stepSimulation()
    #        update_state()
    #        raw_input('Continue?')
    def reverse(self):
        return Trajectory(reversed(self.path))
    def __repr__(self):
        return 't{}'.format(id(self) % 1000)

class Attach(object):
    def __init__(self, robot, arm, grasp, body):
        self.robot = robot
        self.arm = arm
        self.grasp = grasp
        self.body = body
    def step(self):
        link = link_from_name(self.robot, ARM_LINK_NAMES[self.arm])
        gripper_pose = get_link_pose(self.robot, link)
        body_pose = multiply(gripper_pose, self.grasp.value)
        set_pose(self.body, body_pose)
        close_arm(self.robot, self.arm)
    def __repr__(self):
        return '{}({},{},{})'.format(self.__class__.__name__, get_body_name(self.robot),
                                     self.arm, get_body_name(self.body))

class Detach(object):
    def __init__(self, robot, arm, body):
        self.robot = robot
        self.arm = arm
        self.body = body
    def step(self):
        open_arm(self.robot, self.arm)
    def __repr__(self):
        return '{}({},{},{})'.format(self.__class__.__name__, get_body_name(self.robot),
                                     self.arm, get_body_name(self.body))

class Commands(object):
    def __init__(self, commands):
        self.commands = tuple(commands)
    def __repr__(self):
        return 'c{}'.format(id(self) % 1000)

def get_motion_gen(problem):
    def gen(bq1, bq2):
        path = [bq1, bq2]
        #base_path = plan_base_motion(pr2, base_goal)

        bt = Trajectory(path)
        return (bt,)
    return gen

APPROACH_DISTANCE = 0.1

def get_grasp_gen(problem):
    def gen(body):
        grasps = []
        if 'top' in problem.grasp_types:
            approach = (APPROACH_DISTANCE*np.array(TOOL_DIRECTION), unit_quat())
            for grasp in get_top_grasps(body):
                g = Grasp(body, grasp, approach, TOP_HOLDING_LEFT_ARM)
                grasps += [(g,)]
        if 'side' in problem.grasp_types:
            approach = (APPROACH_DISTANCE*np.array(TOOL_DIRECTION), unit_quat())
            for grasp in get_side_grasps(body):
                g = Grasp(body, grasp, approach, SIDE_HOLDING_LEFT_ARM)
                grasps += [(g,)]
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
    fixed = get_fixed_bodies(problem)

    def gen(a, o, p, g):
        gripper_pose = multiply(p.value, invert(g.value)) # w_f_g = w_f_o * (g_f_o)^-1
        approach_pose = multiply(g.approach, gripper_pose)

        link = link_from_name(pr2, LEFT_ARM_LINK) if a == 'left' else link_from_name(pr2, None)
        default_conf = arm_conf(a, g.carry)
        left_joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]

        base_generator = learned_pose_generator(pr2, gripper_pose)
        #base_generator = uniform_pose_generator(pr2, gripper_pose)
        while True:
            set_pose(o, p.value)
            set_joint_positions(pr2, left_joints, default_conf)
            set_pose(pr2, next(base_generator))
            if any(pairwise_collision(pr2, b) for b in fixed):
                continue

            stuff_conf = inverse_kinematics_helper(pr2, link, approach_pose)
            if (stuff_conf is None) or any(pairwise_collision(pr2, b) for b in fixed):
                continue
            approach_conf = get_joint_positions(pr2, left_joints)

            movable_conf = inverse_kinematics_helper(pr2, link, gripper_pose)
            if (movable_conf is None) or any(pairwise_collision(pr2, b) for b in fixed):
                continue
            grasp_conf = get_joint_positions(pr2, left_joints)

            bp = Pose(pr2, get_pose(pr2))
            path = [default_conf, approach_conf, grasp_conf]
            #path = [default_conf, grasp_conf]

            mt = Trajectory(Conf(pr2, left_joints, q) for q in path)
            yield (bp, mt)
    return gen