from __future__ import print_function

import pybullet as p
import random
import time
import numpy as np

from examples.pybullet.utils.pybullet_tools.utils import link_from_name, BodySaver, get_aabb
from .pr2_utils import TOP_HOLDING_LEFT_ARM, SIDE_HOLDING_LEFT_ARM, \
    get_carry_conf, get_top_grasps, get_side_grasps, close_arm, open_arm, arm_conf, get_gripper_link, get_arm_joints, \
    learned_pose_generator, TOOL_DIRECTION, PR2_TOOL_FRAMES, get_x_presses, PR2_GROUPS, joints_from_names, \
    ARM_NAMES, is_drake_pr2, get_group_joints, set_group_conf, get_group_conf
from .utils import invert, multiply, get_name, set_pose, get_link_pose, link_from_name, BodySaver, \
    pairwise_collision, set_joint_positions, get_joint_positions, sample_placement, get_pose, waypoints_from_path, \
    unit_quat, plan_base_motion, plan_joint_motion, base_values_from_pose, pose_from_base_values, \
    uniform_pose_generator, sub_inverse_kinematics, add_fixed_constraint, remove_debug, point_from_pose, \
    remove_fixed_constraint, enable_real_time, disable_real_time, enable_gravity, joint_controller_hold, \
    get_min_limit, user_input, step_simulation, update_state, get_body_name, get_bodies, BASE_LINK, \
    add_segments, unit_pose, set_base_values, get_max_limit
from .pr2_problems import get_fixed_bodies

BASE_EXTENT = 3.5 # 2.5
BASE_LIMITS = (-BASE_EXTENT*np.ones(2), BASE_EXTENT*np.ones(2))
PLAN_JOINTS = True

def get_base_limits(robot):
    if PLAN_JOINTS and is_drake_pr2(robot):
        joints = get_group_joints(robot, 'base')[:2]
        lower = [get_min_limit(robot, j) for j in joints]
        upper = [get_max_limit(robot, j) for j in joints]
        return lower, upper
    return BASE_LIMITS

##################################################

class Pose(object):
    #def __init__(self, position, orientation):
    #    self.position = position
    #    self.orientation = orientation
    def __init__(self, body, value):
        self.body = body
        self.value = tuple(value)
    def step(self):
        set_pose(self.body, self.value)
    def to_base_conf(self):
        value = base_values_from_pose(self.value)
        return Conf(self.body, range(len(value)), value)
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

#####################################

class Command(object):
    def control(self, real_time=False, dt=0):
        raise NotImplementedError()
    def step(self):
        raise NotImplementedError()
    def apply(self, state, **kwargs):
        raise NotImplementedError()

class Trajectory(Command):
    _draw = True
    def __init__(self, path):
        self.path = tuple(path)
        link = BASE_LINK
        #link = 1
        self.points = self.to_points(link) if self._draw else None
        # TODO: constructor that takes in this info
    def apply(self, state, time_step=None, simulate=False, sample=1):
        handles = [] if self.points is None else add_segments(self.points)
        #user_input('Execute?')
        for conf in self.path[1::sample]:
            conf.step()
            for attach in state.attachments.values():
                attach.step()
            update_state()
            if simulate:
                step_simulation()
            if time_step is None:
                user_input('Continue?')
            else:
                time.sleep(time_step)
        end_conf = self.path[-1]
        if isinstance(end_conf, Pose):
            state.poses[end_conf.body] = end_conf
        for handle in handles:
            remove_debug(handle)
    def control(self, real_time=False, dt=0):
        # TODO: just waypoints
        if real_time:
            enable_real_time()
        else:
            disable_real_time()
        for conf in self.path:
            if isinstance(conf, Pose):
                conf = conf.to_base_conf()
            for _ in joint_controller_hold(conf.body, conf.joints, conf.values):
                enable_gravity()
                if not real_time:
                    step_simulation()
                time.sleep(dt)
    def to_points(self, link=BASE_LINK):
        points = []
        for conf in self.path:
            with BodySaver(conf.body):
                conf.step()
                #point = np.array(point_from_pose(get_link_pose(conf.body, link)))
                point = np.array(get_group_conf(conf.body, 'base'))
                point[2] = 0
                point += 1e-2*np.array([0, 0, 1])
                if not (points and np.allclose(points[-1], point, atol=1e-3, rtol=0)):
                    points.append(point)
        points = get_target_path(self)
        return waypoints_from_path(points)
    def reverse(self):
        return Trajectory(reversed(self.path))
    #def __repr__(self):
    #    return 't{}'.format(id(self) % 1000)
    def __repr__(self):
        d = 0
        if self.path:
            conf = self.path[0]
            d = 3 if isinstance(conf, Pose) else len(conf.joints)
        return 't({},{})'.format(d, len(self.path))

def create_trajectory(robot, joints, path):
    return Trajectory(Conf(robot, joints, q) for q in path)

##################################################

class Attach(Command):
    vacuum = True
    def __init__(self, robot, arm, grasp, body):
        self.robot = robot
        self.arm = arm
        self.grasp = grasp
        self.body = body
        self.link = link_from_name(self.robot, PR2_TOOL_FRAMES[self.arm])
    def apply(self, state, **kwargs):
        state.attachments[self.body] = self
        del state.poses[self.body]
    def step(self):
        gripper_pose = get_link_pose(self.robot, self.link)
        body_pose = multiply(gripper_pose, self.grasp.value)
        set_pose(self.body, body_pose)
        if self.arm in ARM_NAMES: # TODO: do I want this?
            close_arm(self.robot, self.arm)
    def control(self, **kwargs):
        if self.vacuum:
            #add_fixed_constraint(self.body, self.robot, self.link)
            add_fixed_constraint(self.body, self.robot, self.link, max_force=1) # Less force makes it easier to pick
            #for _ in range(10):
            #    p.stepSimulation()
        else:
            # TODO: the gripper doesn't quite work yet
            gripper_name = '{}_gripper'.format(self.arm)
            joints = joints_from_names(self.robot, PR2_GROUPS[gripper_name])
            #values = [get_max_limit(self.robot, joint) for joint in joints] # Open
            values = [get_min_limit(self.robot, joint) for joint in joints] # Closed
            for _ in joint_controller_hold(self.robot, joints, values):
                enable_gravity()
                step_simulation()
                #if not real_time:
                #    p.stepSimulation()
                #time.sleep(dt)
    def __repr__(self):
        return '{}({},{},{})'.format(self.__class__.__name__, get_body_name(self.robot),
                                     self.arm, get_name(self.body))

class Detach(Command):
    def __init__(self, robot, arm, body):
        self.robot = robot
        self.arm = arm
        self.body = body
        self.link = link_from_name(self.robot, PR2_TOOL_FRAMES[self.arm])
        # TODO: pose argument to maintain same object
    def apply(self, state, **kwargs):
        del state.attachments[self.body]
        state.poses[self.body] = Pose(self.body, get_pose(self.body))
    def step(self):
        if self.arm in ARM_NAMES: # TODO: do I want this?
            open_arm(self.robot, self.arm)
    def control(self, **kwargs):
        remove_fixed_constraint(self.body, self.robot, self.link)
    def __repr__(self):
        return '{}({},{},{})'.format(self.__class__.__name__, get_body_name(self.robot),
                                     self.arm, get_name(self.body))

##################################################

class Clean(Command):
    def __init__(self, body):
        self.body = body
    def apply(self, state, **kwargs):
        state.cleaned.add(self.body)
        self.step()
    def step(self):
        p.addUserDebugText('Cleaned', textPosition=(0, 0, .25), textColorRGB=(0,0,1), #textSize=1,
                           lifeTime=0, parentObjectUniqueId=self.body)
        #p.setDebugObjectColor(self.body, 0, objectDebugColorRGB=(0,0,1))
    control = step
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.body)

class Cook(Command):
    # TODO: global state here?
    def __init__(self, body):
        self.body = body
    def apply(self, state, **kwargs):
        state.cleaned.remove(self.body)
        state.cooked.add(self.body)
        self.step()
    def step(self):
        # changeVisualShape
        # setDebugObjectColor
        #p.removeUserDebugItem # TODO: remove cleaned
        p.addUserDebugText('Cooked', textPosition=(0, 0, .5), textColorRGB=(1,0,0), #textSize=1,
                           lifeTime=0, parentObjectUniqueId=self.body)
        #p.setDebugObjectColor(self.body, 0, objectDebugColorRGB=(1,0,0))
    control = step
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.body)

#class Commands(object):
#    def __init__(self, commands):
#        self.commands = tuple(commands)
#    def __repr__(self):
#        return 'c{}'.format(id(self) % 1000)

##################################################

# TODO: context aware
def get_motion_gen(problem, teleport=False):
    robot = problem.robot
    fixed = get_fixed_bodies(problem)
    def fn(bq1, bq2):
        if teleport:
            path = [bq1, bq2]
        elif not is_drake_pr2(robot):
            set_pose(robot, bq1.value)
            #start_conf = base_values_from_pose(bq1.value)
            goal_conf = base_values_from_pose(bq2.value)
            raw_path = plan_base_motion(robot, goal_conf, BASE_LIMITS, obstacles=fixed)
            if raw_path is None:
                print('Failed motion plan!')
                return None
            path = [Pose(robot, pose_from_base_values(q, bq1.value)) for q in raw_path]
        else:
            base_joints = get_group_joints(robot, 'base')
            if PLAN_JOINTS:
                # set_pose(robot, unit_pose())
                # set_group_conf(robot, 'base', start_conf)
                set_joint_positions(robot, base_joints, bq1.values)
                raw_path = plan_joint_motion(robot, base_joints, bq2.values,
                                             obstacles=fixed, self_collisions=False)
            else:
                with BodySaver(robot):
                    set_base_values(robot, bq1.values)
                    set_joint_positions(robot, base_joints, np.zeros(len(base_joints)))
                    raw_path = plan_base_motion(robot, bq2.values, BASE_LIMITS, obstacles=fixed)
            if raw_path is None:
                print('Failed motion plan!')
                return None
            #path = [Pose(robot, pose_from_base_values(q, bq1.value)) for q in raw_path]
            path = [Conf(robot, base_joints, q) for q in raw_path]
        bt = Trajectory(path)
        return (bt,)
    return fn

##################################################

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

##################################################

def get_ik_ir_gen(problem, max_attempts=25, learned=True, teleport=False):
    robot = problem.robot
    fixed = get_fixed_bodies(problem)

    def gen(a, o, p, g):
        gripper_pose = multiply(p.value, invert(g.value)) # w_f_g = w_f_o * (g_f_o)^-1
        approach_pose = multiply(g.approach, gripper_pose)

        link = get_gripper_link(robot, a)
        default_conf = arm_conf(a, g.carry)
        arm_joints = get_arm_joints(robot, a)
        base_joints = get_group_joints(robot, 'base')

        if learned:
            base_generator = learned_pose_generator(robot, gripper_pose, arm=a, grasp_type=g.grasp_type)
        else:
            base_generator = uniform_pose_generator(robot, gripper_pose)
        while True:
            for _ in range(max_attempts):
                set_pose(o, p.value)
                set_joint_positions(robot, arm_joints, default_conf)
                base_conf = next(base_generator)
                #base_pose = next(base_generator)
                #set_pose(robot, base_pose)
                set_joint_positions(robot, base_joints, base_conf)
                if any(pairwise_collision(robot, b) for b in fixed):
                    continue

                approach_movable_conf = sub_inverse_kinematics(robot, arm_joints[0], link, approach_pose)
                #approach_movable_conf = inverse_kinematics(robot, link, approach_pose)
                if (approach_movable_conf is None) or any(pairwise_collision(robot, b) for b in fixed):
                    continue
                approach_conf = get_joint_positions(robot, arm_joints)

                grasp_movable_conf = sub_inverse_kinematics(robot, arm_joints[0], link, gripper_pose)
                #grasp_movable_conf = inverse_kinematics(robot, link, gripper_pose)
                if (grasp_movable_conf is None) or any(pairwise_collision(robot, b) for b in fixed):
                    continue
                grasp_conf = get_joint_positions(robot, arm_joints)
                #bq = Pose(robot, get_pose(robot))
                bq = Conf(robot, base_joints, base_conf)

                if teleport:
                    path = [default_conf, approach_conf, grasp_conf]
                else:
                    #set_joint_positions(robot, arm_joints, approach_conf)
                    control_path = plan_joint_motion(robot, arm_joints, approach_conf,
                                                     obstacles=fixed, self_collisions=False, direct=True)
                    set_joint_positions(robot, arm_joints, approach_conf)
                    retreat_path = plan_joint_motion(robot, arm_joints, default_conf,
                                                     obstacles=fixed, self_collisions=False)
                    path = retreat_path[::-1] + control_path[::-1]
                mt = Trajectory(Conf(robot, arm_joints, q) for q in path)
                yield (bq, mt)
                break
            else:
                yield None
    return gen

##################################################

def get_press_gen(problem, max_attempts=25, learned=True, teleport=False):
    robot = problem.robot
    fixed = get_fixed_bodies(problem)

    def gen(arm, button):
        fixed_wo_button = list(filter(lambda b: b != button, fixed))
        pose = get_pose(button)
        grasp_type = 'side'

        link = get_gripper_link(robot, arm)
        default_conf = get_carry_conf(arm, grasp_type)
        joints = get_arm_joints(robot, arm)

        presses = get_x_presses(button)
        approach = ((APPROACH_DISTANCE, 0, 0), unit_quat())
        while True:
            for _ in range(max_attempts):
                press_pose = random.choice(presses)
                gripper_pose = multiply(pose, invert(press_pose)) # w_f_g = w_f_o * (g_f_o)^-1
                #approach_pose = gripper_pose # w_f_g * g_f_o * o_f_a = w_f_a
                approach_pose = multiply(gripper_pose, invert(multiply(press_pose, approach)))

                if learned:
                    base_generator = learned_pose_generator(robot, gripper_pose, arm=arm, grasp_type=grasp_type)
                else:
                    base_generator = uniform_pose_generator(robot, gripper_pose)
                set_joint_positions(robot, joints, default_conf)
                set_pose(robot, next(base_generator))
                raise NotImplementedError('Need to change this')
                if any(pairwise_collision(robot, b) for b in fixed):
                    continue

                approach_movable_conf = sub_inverse_kinematics(robot, joints[0], link, approach_pose)
                #approach_movable_conf = inverse_kinematics(robot, link, approach_pose)
                if (approach_movable_conf is None) or any(pairwise_collision(robot, b) for b in fixed):
                    continue
                approach_conf = get_joint_positions(robot, joints)

                gripper_movable_conf = sub_inverse_kinematics(robot, joints[0], link, gripper_pose)
                #gripper_movable_conf = inverse_kinematics(robot, link, gripper_pose)
                if (gripper_movable_conf is None) or any(pairwise_collision(robot, b) for b in fixed_wo_button):
                    continue
                grasp_conf = get_joint_positions(robot, joints)
                bp = Pose(robot, get_pose(robot))

                if teleport:
                    path = [default_conf, approach_conf, grasp_conf]
                else:
                    control_path = plan_joint_motion(robot, joints, approach_conf,
                                                     obstacles=fixed_wo_button, self_collisions=False, direct=True)
                    if control_path is None: continue
                    set_joint_positions(robot, joints, approach_conf)
                    retreat_path = plan_joint_motion(robot, joints, default_conf,
                                                     obstacles=fixed, self_collisions=False)
                    if retreat_path is None: continue
                    path = retreat_path[::-1] + control_path[::-1]
                mt = Trajectory(Conf(robot, joints, q) for q in path)
                yield (bp, mt)
                break
            else:
                yield None
    return gen

#####################################

def step_commands(commands, time_step=None, simulate=False):
    # update_state()
    #if simulate: step_simulation()
    user_input('Execute?')
    attachments = {}
    for i, command in enumerate(commands):
        print(i, command)
        if type(command) is Attach:
            attachments[command.body] = command
        elif type(command) is Detach:
            del attachments[command.body]
        elif type(command) is Trajectory:
            # for conf in command.path:
            for conf in command.path[1:]:
                conf.step()
                for attach in attachments.values():
                    attach.step()
                update_state()
                # print attachments
                if simulate:
                    step_simulation()
                if time_step is None:
                    user_input('Continue?')
                else:
                    time.sleep(time_step)
        elif type(command) in [Clean, Cook]:
            command.step()
        else:
            raise ValueError(command)


def control_commands(commands):
    user_input('Control?')
    for i, command in enumerate(commands):
        print(i, command)
        command.control()

#####################################

class State(object):
    def __init__(self, attachments={}, cleaned=set(), cooked=set()):
        self.poses = {body: Pose(body, get_pose(body)) for body in get_bodies() if body not in attachments}
        self.attachments = attachments
        self.cleaned = cleaned
        self.cooked = cooked

def apply_commands(state, commands, **kwargs):
    user_input('Apply?')
    for i, command in enumerate(commands):
        print(i, command)
        command.apply(state, **kwargs)
        #user_input('Apply?')

#####################################

def get_target_point(conf):
    # TODO: use full body aabb
    robot = conf.body
    link = link_from_name(robot, 'torso_lift_link')
    #link = BASE_LINK
    # TODO: center of mass instead?
    # TODO: look such that cone bottom touches at bottom
    # TODO: the target isn't the center which causes it to drift
    with BodySaver(conf.body):
        conf.step()
        lower, upper = get_aabb(robot, link)
        center = np.average([lower, upper], axis=0)
        point = np.array(get_group_conf(conf.body, 'base'))
        #point[2] = upper[2]
        point[2] = center[2]
        #center, _ = get_center_extent(conf.body)
        return point


def get_target_path(trajectory):
    # TODO: only do bounding boxes for moving links on the trajectory
    target_path = []
    for conf in trajectory.path:
        # TODO: could draw the target point path sequence
        target_path.append(get_target_point(conf))
    #add_segments(target_path)
    return target_path