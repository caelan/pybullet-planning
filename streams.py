from pybullet_utils import invert, multiply, get_body_name, set_pose, get_link_pose, link_from_name, \
    pairwise_collision, set_joint_positions, get_joint_positions, sample_placement, get_pose, \
    unit_quat
from pr2_utils import TOP_HOLDING_LEFT_ARM, SIDE_HOLDING_LEFT_ARM, get_carry_conf, \
    get_top_grasps, get_side_grasps, close_arm, open_arm, arm_conf, get_gripper_link, get_arm_joints, \
    inverse_kinematics, inverse_kinematics_helper, learned_pose_generator, TOOL_DIRECTION, ARM_LINK_NAMES, get_x_presses
from problems import get_fixed_bodies

import pybullet as p
import numpy as np
import random

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
    def __init__(self, grasp_type, body, value, approach, carry):
        self.grasp_type = grasp_type
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

class Clean(object):
    def __init__(self, body):
        self.body = body
    def step(self):
        p.addUserDebugText('Cleaned', textPosition=(0, 0, .25), textColorRGB=(0,0,1), #textSize=1,
                           lifeTime=0, parentObjectUniqueId=self.body)
        p.setDebugObjectColor(self.body, 0, objectDebugColorRGB=(0,0,1))
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.body)

class Cook(object):
    # TODO: global state here?
    def __init__(self, body):
        self.body = body
    def step(self):
        # changeVisualShape
        # setDebugObjectColor
        #p.removeUserDebugItem # TODO: remove cleaned
        p.addUserDebugText('Cooked', textPosition=(0, 0, .5), textColorRGB=(1,0,0), #textSize=1,
                           lifeTime=0, parentObjectUniqueId=self.body)
        #p.setDebugObjectColor(self.body, 0, objectDebugColorRGB=(1,0,0))
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.body)

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

def get_grasp_gen(problem, randomize=True):
    def fn(body):
        grasps = []
        if 'top' in problem.grasp_types:
            approach = (APPROACH_DISTANCE*np.array(TOOL_DIRECTION), unit_quat())
            for grasp in get_top_grasps(body):
                g = Grasp('top', body, grasp, approach, TOP_HOLDING_LEFT_ARM)
                grasps += [(g,)]
        if 'side' in problem.grasp_types:
            # TODO: change this tool direction
            approach = (APPROACH_DISTANCE*np.array(TOOL_DIRECTION), unit_quat())
            for grasp in get_side_grasps(body):
                g = Grasp('side', body, grasp, approach, SIDE_HOLDING_LEFT_ARM)
                grasps += [(g,)]
        if randomize:
            random.shuffle(grasps)
        return grasps
    return fn

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

def get_ik_ir_gen(problem, max_attempts=25):
    robot = problem.robot
    fixed = get_fixed_bodies(problem)

    def gen(a, o, p, g):
        gripper_pose = multiply(p.value, invert(g.value)) # w_f_g = w_f_o * (g_f_o)^-1
        approach_pose = multiply(g.approach, gripper_pose)

        link = get_gripper_link(robot, a)
        default_conf = arm_conf(a, g.carry)
        joints = get_arm_joints(robot, a)

        base_generator = learned_pose_generator(robot, gripper_pose, arm=a, grasp_type=g.grasp_type)
        #base_generator = uniform_pose_generator(robot, gripper_pose)
        while True:
            for _ in xrange(max_attempts):
                set_pose(o, p.value)
                set_joint_positions(robot, joints, default_conf)
                set_pose(robot, next(base_generator))
                if any(pairwise_collision(robot, b) for b in fixed):
                    continue

                stuff_conf = inverse_kinematics_helper(robot, link, approach_pose)
                if (stuff_conf is None) or any(pairwise_collision(robot, b) for b in fixed):
                    continue
                approach_conf = get_joint_positions(robot, joints)

                movable_conf = inverse_kinematics_helper(robot, link, gripper_pose)
                if (movable_conf is None) or any(pairwise_collision(robot, b) for b in fixed):
                    continue
                grasp_conf = get_joint_positions(robot, joints)

                bp = Pose(robot, get_pose(robot))
                path = [default_conf, approach_conf, grasp_conf]
                mt = Trajectory(Conf(robot, joints, q) for q in path)
                yield (bp, mt)
                break
            else:
                yield None
    return gen

def get_press_gen(problem, max_attempts=25):
    robot = problem.robot
    fixed = get_fixed_bodies(problem)

    def gen(a, o):
        pose = get_pose(o)
        grasp_type = 'side'

        link = get_gripper_link(robot, a)
        default_conf = get_carry_conf(a, grasp_type)
        joints = get_arm_joints(robot, a)

        presses = get_x_presses(o)
        while True:
            for _ in xrange(max_attempts):
                press = random.choice(presses)
                gripper_pose = multiply(pose, invert(press))
                approach_pose = gripper_pose
                #approach_pose = multiply(g.approach, gripper_pose)

                base_generator = learned_pose_generator(robot, gripper_pose, arm=a, grasp_type=grasp_type)
                set_joint_positions(robot, joints, default_conf)
                set_pose(robot, next(base_generator))
                if any(pairwise_collision(robot, b) for b in fixed):
                    continue

                stuff_conf = inverse_kinematics_helper(robot, link, approach_pose)
                if (stuff_conf is None) or any(pairwise_collision(robot, b) for b in fixed):
                    continue
                approach_conf = get_joint_positions(robot, joints)

                movable_conf = inverse_kinematics_helper(robot, link, gripper_pose)
                if (movable_conf is None) or any(pairwise_collision(robot, b) for b in fixed):
                    continue
                grasp_conf = get_joint_positions(robot, joints)

                bp = Pose(robot, get_pose(robot))
                path = [default_conf, approach_conf, grasp_conf]
                mt = Trajectory(Conf(robot, joints, q) for q in path)
                yield (bp, mt)
                break
            else:
                yield None
    return gen