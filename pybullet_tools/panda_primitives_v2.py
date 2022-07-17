from __future__ import print_function

import copy
import pybullet as p
import random
import time
from itertools import islice, count

import numpy as np

from .ikfast.franka_panda.ik import is_ik_compiled, ikfast_inverse_kinematics, PANDA_LEFT_INFO, bi_panda_inverse_kinematics
from .ikfast.utils import USE_CURRENT, USE_ALL
from .pr2_problems import get_fixed_bodies
from .panda_utils import TOP_HOLDING_LEFT_ARM_CENTERED, TOP_HOLDING_LEFT_ARM, SIDE_HOLDING_LEFT_ARM, GET_GRASPS, get_gripper_joints, \
    get_carry_conf, get_top_grasps, get_side_grasps, open_arm, arm_conf, get_gripper_link, get_arm_joints, \
    learned_pose_generator, PANDA_TOOL_FRAMES, get_x_presses, BI_PANDA_GROUPS, joints_from_names, arm_from_arm,\
    is_drake_pr2, get_group_joints, get_group_conf, compute_grasp_width, PANDA_GRIPPER_ROOTS, get_group_links, \
    are_forces_balanced, get_other_arm, TARGET, PLATE_GRASP_LEFT_ARM
from .utils import invert, multiply, get_name, set_pose, get_link_pose, is_placement, \
    pairwise_collision, set_joint_positions, get_joint_positions, sample_placement, get_pose, waypoints_from_path, \
    unit_quat, plan_base_motion, plan_joint_motion, base_values_from_pose, pose_from_base_values, \
    uniform_pose_generator, sub_inverse_kinematics, add_fixed_constraint, remove_debug, remove_fixed_constraint, \
    disable_real_time, enable_gravity, joint_controller_hold, get_distance, \
    get_min_limit, user_input, step_simulation, get_body_name, get_bodies, BASE_LINK, \
    add_segments, get_max_limit, link_from_name, BodySaver, get_aabb, Attachment, interpolate_poses, \
    plan_direct_joint_motion, has_gui, create_attachment, wait_for_duration, get_extend_fn, set_renderer, \
    get_custom_limits, all_between, get_unit_vector, wait_if_gui, joint_from_name, get_joint_info,\
    set_base_values, euler_from_quat, INF, elapsed_time, get_moving_links, flatten_links, get_relative_pose, get_link_name, \
    get_max_force, compute_jacobian, get_COM, matrix_from_quat, check_overlap, set_point, set_joint_position, get_joint_position, \
    is_pose_on_r, quat_from_euler, set_joint_positions_torque, body_from_name, get_same_relative_pose
import math
BASE_EXTENT = 2.5
BASE_LIMITS = (-BASE_EXTENT*np.ones(2), BASE_EXTENT*np.ones(2))
GRASP_LENGTH = 0.02
APPROACH_DISTANCE = 0.01 + GRASP_LENGTH
SELF_COLLISIONS = False
TABLE = 3

##################################################

def get_base_limits(robot):
    if is_drake_pr2(robot):
        joints = get_group_joints(robot, 'base')[:2]
        lower = [get_min_limit(robot, j) for j in joints]
        upper = [get_max_limit(robot, j) for j in joints]
        return lower, upper
    return BASE_LIMITS

##################################################

class Pose(object):
    num = count()
    #def __init__(self, position, orientation):
    #    self.position = position
    #    self.orientation = orientation
    def __init__(self, body, value=None, support=None, init=False):
        self.body = body
        if value is None:
            value = get_pose(self.body)
        self.value = tuple(value)
        self.support = support
        self.init = init
        self.index = next(self.num)
    @property
    def bodies(self):
        return flatten_links(self.body)
    def assign(self):
        set_pose(self.body, self.value)
    def iterate(self):
        yield self
    def to_base_conf(self):
        values = base_values_from_pose(self.value)
        return Conf(self.body, range(len(values)), values)
    def __repr__(self):
        index = self.index
        #index = id(self) % 1000
        return 'p{}'.format(index)

class Grasp(object):
    def __init__(self, grasp_type, body, value, approach, carry):
        self.grasp_type = grasp_type
        self.body = body
        self.value = tuple(value) # gripper_from_object
        self.approach = tuple(approach)
        self.carry = tuple(carry)
    def get_attachment(self, robot, arm):
        tool_link = link_from_name(robot, PANDA_TOOL_FRAMES[arm])
        return Attachment(robot, tool_link, self.value, self.body)
    def __repr__(self):
        return 'g{}'.format(id(self) % 1000)

class Conf(object):
    def __init__(self, body, joints, values=None, init=False):
        self.body = body
        self.joints = joints
        if values is None:
            values = get_joint_positions(self.body, self.joints)
        self.values = tuple(values)
        self.init = init
    @property
    def bodies(self): # TODO: misnomer
        return flatten_links(self.body, get_moving_links(self.body, self.joints))
    def assign(self):

        set_joint_positions(self.body, self.joints, self.values)
    def iterate(self):
        yield self
    def __repr__(self):
        return 'q{}'.format(id(self) % 1000)

#####################################

class Command(object):
    def control(self, dt=0):
        raise NotImplementedError()
    def apply(self, state, **kwargs):
        raise NotImplementedError()
    def iterate(self):
        raise NotImplementedError()

class Commands(object):
    def __init__(self, state, savers=[], commands=[]):
        self.state = state
        self.savers = tuple(savers)
        self.commands = tuple(commands)
    def assign(self):
        for saver in self.savers:
            saver.restore()
        return copy.copy(self.state)
    def apply(self, state, **kwargs):
        for command in self.commands:
            for result in command.apply(state, **kwargs):
                yield result
    def __repr__(self):
        return 'c{}'.format(id(self) % 1000)

#####################################

def set_joint_force_limits(robot, arm):
    links = get_group_links(robot, arm_from_arm(arm))
    joints = get_group_joints(robot, arm_from_arm(arm))
    for i in range(len(joints)):
        joint = joints[i]
        limits = get_joint_info(robot, joint)
        p.changeDynamics(robot, links[i], jointLimitForce=limits.jointMaxForce)

######################################
class Trajectory(Command):
    _draw = False
    def __init__(self, path):
        self.path = tuple(path)
        # TODO: constructor that takes in this info
    def apply(self, state, sample=1):
        handles = add_segments(self.to_points()) if self._draw and has_gui() else []
        for conf in self.path[::sample]:
            conf.assign()
            yield
        end_conf = self.path[-1]
        if isinstance(end_conf, Pose):
            state.poses[end_conf.body] = end_conf
        for handle in handles:
            remove_debug(handle)
    def control(self, dt=0, **kwargs):
        # TODO: just waypoints
        for conf in self.path:
            if isinstance(conf, Pose):
                conf = conf.to_base_conf()
            for _ in joint_controller_hold(conf.body, conf.joints, conf.values):
                step_simulation()
                time.sleep(dt)
    def to_points(self, link=BASE_LINK):
        # TODO: this is computationally expensive
        points = []
        for conf in self.path:
            with BodySaver(conf.body):
                conf.assign()
                #point = np.array(point_from_pose(get_link_pose(conf.body, link)))
                point = np.array(get_group_conf(conf.body, 'base'))
                point[2] = 0
                point += 1e-2*np.array([0, 0, 1])
                if not (points and np.allclose(points[-1], point, atol=1e-3, rtol=0)):
                    points.append(point)
        points = get_target_path(self)
        return waypoints_from_path(points)
    def distance(self, distance_fn=get_distance):
        total = 0.
        for q1, q2 in zip(self.path, self.path[1:]):
            total += distance_fn(q1.values, q2.values)
        return total
    def iterate(self):
        for conf in self.path:
            yield conf
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

class GripperCommand(Command):
    def __init__(self, robot, arm, position, teleport=False):
        self.robot = robot
        self.arm = arm
        self.position = position
        self.teleport = teleport
    def apply(self, state, **kwargs):
        joints = get_gripper_joints(self.robot, self.arm)
        start_conf = get_joint_positions(self.robot, joints)
        end_conf = [self.position] * len(joints)
        if self.teleport:
            path = [start_conf, end_conf]
        else:
            extend_fn = get_extend_fn(self.robot, joints)
            path = [start_conf] + list(extend_fn(start_conf, end_conf))
        for positions in path:
            set_joint_positions(self.robot, joints, positions)
            yield positions
    def control(self, **kwargs):
        joints = get_gripper_joints(self.robot, self.arm)
        positions = [self.position]*len(joints)
        for _ in joint_controller_hold(self.robot, joints, positions):
            yield

    def __repr__(self):
        return '{}({},{},{})'.format(self.__class__.__name__, get_body_name(self.robot),
                                     self.arm, self.position)
class Attach(Command):
    vacuum = True
    def __init__(self, robot, arm, grasp, body):
        self.robot = robot
        self.arm = arm
        self.grasp = grasp
        self.body = body
        self.link = link_from_name(self.robot, PANDA_TOOL_FRAMES.get(self.arm, self.arm))
        #self.attachment = None
    def assign(self):
        gripper_pose = get_link_pose(self.robot, self.link)
        body_pose = multiply(gripper_pose, self.grasp.value)
        set_pose(self.body, body_pose)
    def apply(self, state, **kwargs):
        state.attachments[self.body] = create_attachment(self.robot, self.link, self.body)
        state.grasps[self.body] = self.grasp
        del state.poses[self.body]
        yield
    def control(self, dt=0, **kwargs):
        if self.vacuum:
            add_fixed_constraint(self.body, self.robot, self.link)
            #add_fixed_constraint(self.body, self.robot, self.link, max_force=1) # Less force makes it easier to pick
        else:
            # TODO: the gripper doesn't quite work yet
            gripper_name = '{}_gripper'.format(self.arm)
            joints = joints_from_names(self.robot, BI_PANDA_GROUPS[gripper_name])
            values = [get_min_limit(self.robot, joint) for joint in joints] # Closed
            for _ in joint_controller_hold(self.robot, joints, values):
                step_simulation()
                time.sleep(dt)
    def __repr__(self):
        return '{}({},{},{})'.format(self.__class__.__name__, get_body_name(self.robot),
                                     self.arm, get_name(self.body))

class Detach(Command):
    def __init__(self, robot, arm, body):
        self.robot = robot
        self.arm = arm
        self.body = body
        self.link = link_from_name(self.robot, PANDA_TOOL_FRAMES.get(self.arm, self.arm))
        # TODO: pose argument to maintain same object
    def apply(self, state, **kwargs):
        del state.attachments[self.body]
        state.poses[self.body] = Pose(self.body, get_pose(self.body))
        del state.grasps[self.body]
        yield
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
        self.control()
        yield
    def control(self, **kwargs):
        p.addUserDebugText('Cleaned', textPosition=(0, 0, .25), textColorRGB=(0,0,1), #textSize=1,
                           lifeTime=0, parentObjectUniqueId=self.body)
        #p.setDebugObjectColor(self.body, 0, objectDebugColorRGB=(0,0,1))
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.body)

class Cook(Command):
    # TODO: global state here?
    def __init__(self, body):
        self.body = body
    def apply(self, state, **kwargs):
        state.cleaned.remove(self.body)
        state.cooked.add(self.body)
        self.control()
        yield
    def control(self, **kwargs):
        # changeVisualShape
        # setDebugObjectColor
        #p.removeUserDebugItem # TODO: remove cleaned
        p.addUserDebugText('Cooked', textPosition=(0, 0, .5), textColorRGB=(1,0,0), #textSize=1,
                           lifeTime=0, parentObjectUniqueId=self.body)
        #p.setDebugObjectColor(self.body, 0, objectDebugColorRGB=(1,0,0))
    def __repr__(self):
        return '{}({})'.format(self.__class__.__name__, self.body)

##################################################

def get_grasp_gen(problem, collisions=False, randomize=True):
    for grasp_type in problem.grasp_types:
        if grasp_type not in GET_GRASPS:
            raise ValueError('Unexpected grasp type:', grasp_type)
    def fn(body):
        # TODO: max_grasps
        # TODO: return grasps one by one
        grasps = []
        arm = 'left'
        #carry_conf = get_carry_conf(arm, 'top')
        print("$$$$$$$$$$$$$$$$$$$$$$$$ grasp gen")
        if 'top' in problem.grasp_types:
            approach_vector = APPROACH_DISTANCE*get_unit_vector([1, 0, 0])
            grasps.extend(Grasp('top', body, g, multiply((approach_vector, unit_quat()), g), TOP_HOLDING_LEFT_ARM_CENTERED)
                          for g in get_top_grasps(body, grasp_length=GRASP_LENGTH))
        if 'side' in problem.grasp_types:
            approach_vector = APPROACH_DISTANCE*get_unit_vector([2, 0, -1])
            grasps.extend(Grasp('side', body, g, multiply((approach_vector, unit_quat()), g), SIDE_HOLDING_LEFT_ARM)
                          for g in get_side_grasps(body, grasp_length=GRASP_LENGTH))
        filtered_grasps = []
        for grasp in grasps:
            grasp_width = compute_grasp_width(problem.robot, arm, body, grasp.value) if collisions else 0.0
            if grasp_width is not None:
                grasp.grasp_width = grasp_width
                filtered_grasps.append(grasp)
        if randomize:
            random.shuffle(filtered_grasps)
        return [(g,) for g in filtered_grasps]
        #for g in filtered_grasps:
        #    yield (g,)
    return fn

##################################################

def accelerate_gen_fn(gen_fn, max_attempts=1):
    def new_gen_fn(*inputs):
        generator = gen_fn(*inputs)
        while True:
            for i in range(max_attempts):
                try:
                    output = next(generator)
                except StopIteration:
                    return
                if output is not None:
                    print(gen_fn.__name__, i)
                    yield output
                    break
    return new_gen_fn

def get_stable_gen(problem, collisions=True, **kwargs):
    obstacles = problem.fixed if collisions else []
    robot = problem.robot
    link = link_from_name(robot, 'l_panda_hand')
    def gen(body, surface):
        # TODO: surface poses are being sampled in pr2_belief
        if surface is None:
            surfaces = problem.surfaces
        else:
            surfaces = [surface]
        while True:
            surface = random.choice(surfaces) # TODO: weight by area
            body_pose = sample_placement(body, surface, **kwargs)
            if body_pose is None:
                break
            p = Pose(body, body_pose, surface)
            p.assign()
            if are_forces_balanced(body, p, surface,robot, link, problem.movable) and not any(pairwise_collision(body, obst) for obst in obstacles if obst not in {body, surface}):
                yield (p,)
    # TODO: apply the acceleration technique here
    return gen

##################################################

def get_tool_from_root(robot, arm):
    root_link = link_from_name(robot, PANDA_GRIPPER_ROOTS[arm])
    tool_link = link_from_name(robot, PANDA_TOOL_FRAMES[arm])
    return get_relative_pose(robot, root_link, tool_link)

def iterate_approach_path(robot, arm, gripper, pose, grasp, body=None):
    tool_from_root = get_tool_from_root(robot, arm)
    grasp_pose = multiply(pose.value, invert(grasp.value))
    approach_pose = multiply(pose.value, invert(grasp.approach))
    for tool_pose in interpolate_poses(grasp_pose, approach_pose):
        set_pose(gripper, multiply(tool_pose, tool_from_root))
        if body is not None:
            set_pose(body, multiply(tool_pose, grasp.value))
        yield

def get_ir_sampler(problem, custom_limits={}, max_attempts=20, collisions=True, learned=True):
    robot = problem.robot
    obstacles = problem.fixed if collisions else []
    gripper = problem.get_gripper()

    def gen_fn(arm, obj, pose, grasp):
        pose.assign()
        approach_obstacles = {obst for obst in obstacles if not is_placement(obj, obst)}
        for _ in iterate_approach_path(robot, arm, gripper, pose, grasp, body=obj):
            if any(pairwise_collision(gripper, b) or pairwise_collision(obj, b) for b in approach_obstacles):
                return
        gripper_pose = multiply(pose.value, invert(grasp.value)) # w_f_g = w_f_o * (g_f_o)^-1
        default_conf = arm_conf(arm, grasp.carry)
        base_joints = get_group_joints(robot, 'base')
        arm_joints = get_arm_joints(robot, arm)
        base_conf = get_pose(robot)
        lower_limits, upper_limits = get_custom_limits(robot, base_joints, custom_limits)
        while True:
            count = 0
            for _ in range (max_attempts):
                count += 1
                # if not all_between(lower_limits, base_conf, upper_limits):
                #     continue
                bq = Conf(robot, base_joints, base_conf)
                pose.assign()
                # bq.assign()
                set_joint_positions(robot, arm_joints, default_conf)
                if any(pairwise_collision(robot, b) for b in obstacles + [obj]):
                    continue
                #print('IR attempts:', count)
                yield (bq,)
                break
            else:
                yield None
    return gen_fn

##################################################
def get_torque_limits_not_exceded_test(problem):
    holding_arm = problem.holding_arm
    robot = problem.robot
    max_limits = []
    if holding_arm is not None:
        l = PANDA_GRIPPER_ROOTS[holding_arm]
        baseLink = 1
        joints = BI_PANDA_GROUPS[arm_from_arm(holding_arm)]
        for joint in joints:
            jointNum = joint_from_name(robot, joint)
            max_limits.append(get_max_force(problem.robot, jointNum))
        basePose = get_link_pose(problem.robot, baseLink)[0]
        EPS = .3
        comR = []
        totalMass = 0
    def test(a, poses = None, ptotalMass = None, pcomR = None):
        if holding_arm is None:
            return True
        Jl, Ja = compute_jacobian(problem.robot, link_from_name(problem.robot, l))
        Jl = np.array(Jl)
        Ja = np.array(Ja)
        J = Jl[:7,:]
        if pcomR is not None and ptotalMass is not None:
            comR = pcomR
            totalMass = ptotalMass
        else:
            comR, totalMass = get_COM(problem.movable, b2=body_from_name(TARGET), poses=poses)

        force = np.array([0, 0, totalMass * -9.8])
        torques = np.matmul(J, force)
        testVal = True
        for i in range(len(torques)):
            if (abs(torques[i]) > (max_limits[i]*EPS)):
                testVal = False
        return testVal
    return test

def get_sample_stable_holding_conf_gen_v2(problem, max_attempts = 100, stepSize = 0.01):
    holding_arm = problem.holding_arm
    robot = problem.robot
    max_limits = []
    l = PANDA_TOOL_FRAMES[holding_arm]
    joints = BI_PANDA_GROUPS[arm_from_arm(problem.holding_arm)]
    jointNums = []
    for joint in joints:
        jointNum = joint_from_name(robot, joint)
        jointNums.append(jointNum)
        max_limits.append(get_max_force(problem.robot, jointNum))
    gripper_link = link_from_name(robot, PANDA_GRIPPER_ROOTS[holding_arm])
    arm_joints = get_arm_joints(robot, holding_arm)
    test_torque_limits = get_torque_limits_not_exceded_test(problem)
    jointStart = get_joint_position(robot, joint_from_name(robot, BI_PANDA_GROUPS[arm_from_arm(holding_arm)][-1]))
    link_start = euler_from_quat(get_link_pose(robot, gripper_link)[1])[1]
    def gen(arm, pcomR, ptotalMass):
        movable = get_objects_on_target(problem)
        attempts = 0
        originalObjPoses = [get_pose(obj) for obj in movable]
        originalPoses = get_joint_positions(problem.robot, jointNums)
        originalTargetPose = Pose(body_from_name(TARGET), get_pose(body_from_name(TARGET)))

        # original gripper pose in world frame
        originalGripperPose = get_link_pose(robot, gripper_link)


        newTargetPose = Pose(body_from_name(TARGET), originalTargetPose.value)
        newGripperPose = originalGripperPose
        newObjectPoses = [pose for pose in originalObjPoses]
        holding_grasp = problem.holding_grasp
        attachment = holding_grasp.get_attachment(robot, holding_arm)
        attachments = {attachment.child: attachment}
        while True:
            if attempts >= max_attempts:
                originalTargetPose.assign()
                set_joint_positions(robot, jointNums, originalPoses)
                for i in range(len(movable)):
                    set_pose(movable[i], originalObjPoses[i])
                return None, None
            attempts += 1
            hold = ((newTargetPose.value[0][0],
                             newTargetPose.value[0][1]+stepSize,
                             newTargetPose.value[0][2]),newTargetPose.value[1])
            newTargetPose.value = hold
            newTargetPose.assign()
            # newTargetPose[0][1] -= stepSize
            newGripperPose = ((newGripperPose[0][0],
                                    newGripperPose[0][1] + stepSize,
                                    newGripperPose[0][2]), newGripperPose[1])
            for i in range(len(newObjectPoses)):
                newObjectPoses[i] = ((newObjectPoses[i][0][0],
                                        newObjectPoses[i][0][1] + stepSize,
                                        newObjectPoses[i][0][2]), newObjectPoses[i][1])
            # gripper_pose = multiply(newGripperPose, invert(problem.holding_grasp.value))
            newJointAngles = bi_panda_inverse_kinematics(robot, holding_arm, gripper_link, newGripperPose, max_attempts=max_attempts, max_time=3.5)
            if newJointAngles is None:
                print("IK failed")
                continue
            set_joint_positions(robot, jointNums, newJointAngles)
            pose_test = get_link_pose(robot, gripper_link)
            if(pose_test[1] != newGripperPose[1]):
                joint_rotation = get_joint_position(robot, joint_from_name(robot, BI_PANDA_GROUPS[arm_from_arm(holding_arm)][-1]))
                gripper_correction = joint_rotation - (euler_from_quat(newGripperPose[1])[1] - euler_from_quat(pose_test[1])[1])
                set_joint_position(robot, joint_from_name(robot, BI_PANDA_GROUPS[arm_from_arm(holding_arm)][-1]), gripper_correction)
            if test_torque_limits(holding_arm, ptotalMass=ptotalMass, pcomR=pcomR):
                resolutions = 0.05**np.ones(len(arm_joints))
                # path = plan_joint_motion(robot, arm_joints, newJointAngles, attachments=attachments.values(),
                #                               obstacles=[], self_collisions=SELF_COLLISIONS,
                #                               resolutions=resolutions,
                #                               restarts=2, iterations=25, smooth=25)
                path = plan_direct_joint_motion(robot, arm_joints, newJointAngles, attachments=attachments.values(),
                                              obstacles=[], self_collisions=SELF_COLLISIONS, custom_limits={},
                                              resolutions=resolutions/2.)
                mt = create_trajectory(robot, arm_joints, path)
                cmd = Commands(State(attachments=attachments), savers=[BodySaver(robot)], commands=[mt])
                shift = newTargetPose.value[0][1] - originalTargetPose.value[0][1]
                print(originalTargetPose)
                print(newTargetPose)
                originalPoses = newJointAngles
                originalTargetPose.value = newTargetPose.value
                originalGripperPose = newGripperPose
                originalTargetPose.assign()
                set_joint_positions(robot, jointNums, originalPoses)
                for i in range(len(movable)):
                    set_pose(movable[i], originalObjPoses[i])
                return mt, shift
    return gen

def get_ik_fn(problem, custom_limits={}, collisions=True, teleport=True):
    robot = problem.robot
    obstacles = problem.fixed if collisions else []
    def fn(arm, obj, pose, grasp, reconfig):
        approach_obstacles = {obst for obst in obstacles if not is_placement(obj, obst)}
        gripper_pose = multiply(pose.value, invert(grasp.value)) # w_f_g = w_f_o * (g_f_o)^-1
        approach_pose = multiply(pose.value, invert(grasp.approach))
        arm_link = get_gripper_link(robot, arm)
        arm_joints = get_arm_joints(robot, arm)

        default_conf = arm_conf(arm, grasp.carry)
        pose.assign()
        open_arm(robot, arm)
        set_joint_positions(robot, arm_joints, default_conf) # default_conf | sample_fn()
        ikfaskt_info = PANDA_LEFT_INFO
        gripper_link = link_from_name(robot, PANDA_GRIPPER_ROOTS[arm])
        grasp_conf = bi_panda_inverse_kinematics(robot, arm, arm_link, gripper_pose, max_attempts=100, max_time=3.5, obstacles=obstacles)
        if (grasp_conf is None) or any(pairwise_collision(robot, b) for b in obstacles): # [obj]
            print('Grasp IK failure', grasp_conf)
            return None
        approach_conf = sub_inverse_kinematics(robot, arm_joints[0], arm_link, approach_pose, custom_limits=custom_limits)
        if (approach_conf is None) or any(pairwise_collision(robot, b) for b in obstacles + [obj]):
            if(any(pairwise_collision(robot, b) for b in obstacles + [obj])):
                print("collision failure")
            print('Approach IK failure', grasp_conf)
            #wait_if_gui()
            return None
        approach_conf = get_joint_positions(robot, arm_joints)
        attachment = grasp.get_attachment(problem.robot, arm)
        attachments = {attachment.child: attachment}
        if teleport:
            path = [default_conf, approach_conf, grasp_conf]
        else:
            resolutions = 0.05**np.ones(len(arm_joints))
            grasp_path = plan_direct_joint_motion(robot, arm_joints, grasp_conf, attachments=attachments.values(),
                                                  obstacles=approach_obstacles, self_collisions=SELF_COLLISIONS,
                                                  custom_limits={}, resolutions=resolutions/2.)
            if grasp_path is None:
                print('Grasp path failure')
                return None
            set_joint_positions(robot, arm_joints, default_conf)
            approach_path = plan_joint_motion(robot, arm_joints, approach_conf, attachments=attachments.values(),
                                              obstacles=obstacles, self_collisions=SELF_COLLISIONS,
                                              custom_limits=custom_limits, resolutions=resolutions,
                                              restarts=4, iterations=25, smooth=25)
            if approach_path is None:
                print('Approach path failure')
                return None
            path = approach_path + grasp_path
        mt = create_trajectory(robot, arm_joints, path)
        if reconfig is not None:
            print("################### reconfig is not none")
            cmd = Commands(State(attachments=attachments), savers=[BodySaver(robot)], commands=[reconfig, mt])
        else:
            print("##################### reconfig is none")
            cmd = Commands(State(attachments=attachments), savers=[BodySaver(robot)], commands=[mt])

        return (cmd,)
    return fn

##################################################
poses = {}
def get_ik_ir_gen(problem, max_attempts=25, learned=True, teleport=False, **kwargs):
    # TODO: compose using general fn
    holding_arm = problem.holding_arm
    ir_sampler = get_ir_sampler(problem, learned=learned, max_attempts=1, **kwargs)
    ik_fn = get_ik_fn(problem, teleport=teleport, **kwargs)
    if holding_arm is not None:
        torque_test = get_torque_limits_not_exceded_test(problem)
        stepSize = 0.0001
        safe_conf_fn = get_sample_stable_holding_conf_gen_v2(problem, stepSize=stepSize, max_attempts=max_attempts)
        comR = []

        gripper_link = link_from_name(problem.robot, PANDA_GRIPPER_ROOTS[problem.holding_arm])
        plate_width = problem.target_width
        joints = get_arm_joints(problem.robot, problem.holding_arm)
    def gen(*inputs):
        a, b, p, g = inputs
        attempts = 0
        if holding_arm is not None:
            global poses
            originalRobot = get_joint_positions(problem.robot, joints)
            originalTargetPose = Pose(body_from_name(TARGET), get_pose(body_from_name(TARGET)))
            originalObjectPs = poses
        # [obj] = Pose(obj, poses[obj])
        originalP = p
        while True:
            reconfig = None
            if max_attempts <= attempts:
                if not p.init:
                    return
                attempts = 0
                yield None
            attempts += 1
            if holding_arm is not None:
                target_overlap = is_pose_on_r(p.value, body_from_name(TARGET))
                if (target_overlap):
                    poses[b] = p.value
                comR, totalMass = get_COM(bodies = [], poses=poses, b2=body_from_name(TARGET))
                if (not torque_test(a, poses, pcomR=comR, ptotalMass=totalMass)):
                    reconfig, shift = safe_conf_fn(a, pcomR=comR, ptotalMass=totalMass)
                    if reconfig is None:
                        continue
                if reconfig is not None:
                    poseOnR = {}
                    for obj in poses:
                        if obj != b and is_pose_on_r(poses[obj], body_from_name(TARGET)):
                            poseOnR[obj] = poses[obj]
                    gripperPose = get_link_pose(problem.robot, gripper_link)
                    newTargetPos = Pose(body_from_name(TARGET), get_pose(body_from_name(TARGET)))
                    pose=((newTargetPos.value[0][0], gripperPose[0][1] - (plate_width / 2 + 0.07) , newTargetPos.value[0][2]), newTargetPos.value[1])
                    prevPose = newTargetPos.value
                    for obj in poseOnR:
                        new_pose = Pose(obj, get_same_relative_pose(p.value, prevPose, pose))
                        new_pose.assign()
                    newTargetPos.value = pose

                    if target_overlap:
                        new_pose = get_same_relative_pose(p.value, prevPose, pose)
                        p.value = new_pose
                        poses[b] = p.value
                        p.assign()
                    newTargetPos.assign()

            ir_generator = ir_sampler(a,b,p,g)

            try:
                ir_outputs = next(ir_generator)
            except StopIteration:
                return
            if ir_outputs is None:
                if holding_arm is not None:
                    originalP.assign()
                    originalTargetPose.assign()
                    poses = {}
                    for obj in originalObjectPs:
                        tempP = Pose(obj, originalObjectPs[obj])
                        tempP.assign()
                        poses[obj] = originalObjectPs[obj]
                    set_joint_positions(problem.robot, joints, originalRobot)
                continue
            ik_outputs = ik_fn(a, b, p, g, reconfig)
            if ik_outputs is None:
                if holding_arm is not None:
                    originalP.assign()
                    originalTargetPose.assign()
                    poses = {}
                    for obj in originalObjectPs:
                        tempP = Pose(obj, originalObjectPs[obj])
                        tempP.assign()
                        poses[obj] = originalObjectPs[obj]
                    set_joint_positions(problem.robot, joints, originalRobot)
                continue
            print('IK attempts:', attempts)
            yield ir_outputs + ik_outputs
            return
            #if not p.init:
            #    return
    return gen

##################################################

def get_motion_gen(problem, custom_limits={}, collisions=True, teleport=False):
    # TODO: include fluents
    robot = problem.robot
    saver = BodySaver(robot)
    obstacles = problem.fixed if collisions else []
    def fn(bq1, bq2, fluents=[]):
        saver.restore()
        bq1.assign()
        if teleport:
            path = [bq1, bq2]
        elif is_drake_pr2(robot):
            raw_path = plan_joint_motion(robot, bq2.joints, bq2.values, attachments=[],
                                         obstacles=obstacles, custom_limits=custom_limits, self_collisions=SELF_COLLISIONS,
                                         restarts=4, iterations=50, smooth=50)
            if raw_path is None:
                print('Failed motion plan!')
                #set_renderer(True)
                #for bq in [bq1, bq2]:
                #    bq.assign()
                #    wait_if_gui()
                return None
            path = [Conf(robot, bq2.joints, q) for q in raw_path]
        else:
            goal_conf = base_values_from_pose(bq2.value)
            raw_path = plan_base_motion(robot, goal_conf, BASE_LIMITS, obstacles=obstacles)
            if raw_path is None:
                print('Failed motion plan!')
                return None
            path = [Pose(robot, pose_from_base_values(q, bq1.value)) for q in raw_path]
        bt = Trajectory(path)
        cmd = Commands(State(), savers=[BodySaver(robot)], commands=[bt])
        return (cmd,)
    return fn

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
                bp = Pose(robot, get_pose(robot)) # TODO: don't use this

                if teleport:
                    path = [default_conf, approach_conf, grasp_conf]
                else:
                    control_path = plan_direct_joint_motion(robot, joints, approach_conf,
                                                     obstacles=fixed_wo_button, self_collisions=SELF_COLLISIONS)
                    if control_path is None: continue
                    set_joint_positions(robot, joints, approach_conf)
                    retreat_path = plan_joint_motion(robot, joints, default_conf,
                                                     obstacles=fixed, self_collisions=SELF_COLLISIONS)
                    if retreat_path is None: continue
                    path = retreat_path[::-1] + control_path[::-1]
                mt = Trajectory(Conf(robot, joints, q) for q in path)
                yield (bp, mt)
                break
            else:
                yield None
    return gen

#####################################

def control_commands(commands, robot = 1, **kwargs):
    # wait_if_gui('Control?')
    disable_real_time()
    enable_gravity()

    for i, command in enumerate(commands):
        print(i, command)
        command.control(*kwargs)


class State(object):
    def __init__(self, attachments={}, cleaned=set(), cooked=set()):
        self.poses = {body: Pose(body, get_pose(body))
                      for body in get_bodies() if body not in attachments}
        self.grasps = {}
        self.attachments = attachments
        self.cleaned = cleaned
        self.cooked = cooked
        self.bodies = get_bodies()
        self.body_names = [get_body_name(body) for body in self.bodies]


    def assign(self):
        for attachment in self.attachments.values():
            #attach.attachment.assign()
            attachment.assign()

def apply_commands(state, commands, time_step=None, pause=False, **kwargs):
    #wait_if_gui('Apply?')
    prev_joints = ()
    for i, command in enumerate(commands):
        print(i, command)
        for j, _ in enumerate(command.apply(state, **kwargs)):
            state.assign()
            jointNums = []
            joints = BI_PANDA_GROUPS[arm_from_arm('left')]
            for joint in joints:
                jointNums.append(joint_from_name(1, joint))
            jointPos = get_joint_positions(1, jointNums)
            set_joint_positions_torque(1, jointNums, jointPos)
            # if prev_joints != jointPos:
            #     initial_conf = PLATE_GRASP_LEFT_ARM
            #     plate_width = 0.2063
            #     l_hand_link = link_from_name(1, 'l_panda_hand')
            #     hand_pose = get_link_pose(1, l_hand_link)
            #     pose=((hand_pose[0][0] + ((plate_width / 2 + 0.07)*math.cos(initial_conf[0])), hand_pose[0][1] + ((plate_width / 2 + 0.07) *math.sin(initial_conf[0])), hand_pose[0][2]),quat_from_euler((0,0,initial_conf[0])))
            #     set_pose(TARGET, pose)
            if j == 0:
                continue
            if time_step is None:
                wait_for_duration(1e-2)
                wait_if_gui('Command {}, Step {}) Next?'.format(i, j))
            else:
                wait_for_duration(time_step)
        if pause:
            wait_if_gui()

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
        conf.assign()
        lower, upper = get_aabb(robot, link)
        center = np.average([lower, upper], axis=0)
        point = np.array(get_group_conf(conf.body, 'base'))
        #point[2] = upper[2]
        point[2] = center[2]
        #center, _ = get_center_extent(conf.body)
        return point


def get_target_path(trajectory):
    # TODO: only do bounding boxes for moving links on the trajectory
    return [get_target_point(conf) for conf in trajectory.path]


def get_objects_on_target(problem, target = TARGET, poses = None):
    obs = []
    target = body_from_name(target)
    if poses is not None:
        for obj in poses:
            if check_overlap(poses[obj][0], target):
                obs.append(obj)
    else:
        for obj in problem.movable:
            if check_overlap(get_pose(obj)[0], target):
                obs.append(obj)
    return obs

def get_sample_stable_holding_conf_gen(problem, max_attempts = 25, stepSize = 0.1):
    holding_arm = problem.holding_arm
    robot = problem.robot
    max_limits = []
    l = PANDA_TOOL_FRAMES[holding_arm]
    joints = BI_PANDA_GROUPS[arm_from_arm(problem.holding_arm)]
    jointNums = []
    for joint in joints:
        jointNum = joint_from_name(robot, joint)
        jointNums.append(jointNum)
        max_limits.append(get_max_force(problem.robot, jointNum))
    gripper_link = get_gripper_link(robot, holding_arm)
    arm_joints = get_arm_joints(robot, holding_arm)
    test_torque_limits = get_torque_limits_not_exceded_test(problem)
    def gen(arm, totalMass = None, comR = None, poses = None):
        originalPoses = get_joint_positions(robot, jointNums)
        movable = get_objects_on_target(problem, poses)
        attempts = 0
        originalObjPoses = [get_pose(obj) for obj in movable]

        originalTargetPose = get_pose(body_from_name(TARGET))
        originalGripperPose = get_link_pose(robot, gripper_link)
        newTargetPose = list(originalTargetPose.copy())
        newGripperPose = list(originalGripperPose.copy())
        newObjectPoses = [list(pose.copy()) for pose in originalObjPoses]
        holding_grasp = problem.holding_grasp
        attachment = holding_grasp.get_attachment(robot, holding_arm)
        attachments = {attachment.child: attachment}
        while True:
            print("$$$$$$$ in stable conf gen")
            if attempts >= max_attempts:
                set_pose(body_from_name(TARGET), originalTargetPose)
                set_joint_positions(robot, jointNums, originalPoses)
                for i in range(len(movabe)):
                    set_pose(movable[i], originalObjPoses[i])
                return
            attempts += 1
            newTargetPose[0][1] -= stepSize
            newGripperPose[0][1] -= stepSize
            for i in range(len(newObjectPoses)):
                newObjectPoses[i][0][1] -= stepSize
            newJointAngles = bi_panda_inverse_kinematics(robot, holding_arm , gripper_link, newGripperPose)
            set_joint_positions(robot, jointNums, newJointAngles)
            if test_torque_limits(arm):
                resolutions = 0.05**np.ones(len(arm_joints))
                path = plan_joint_motion(robot, arm_joints, newJointAngles, attachments=attachments.values(),
                                              obstacles=[], self_collisions=SELF_COLLISIONS,
                                              resolutions=resolutions,
                                              restarts=2, iterations=25, smooth=25)
                mt = create_trajectory(robot, arm_joints, path)
                cmd = Commands(State(attachments=attachments), savers=[BodySaver(robot)], commands=[mt])
                set_pose(body_from_name(TARGET), originalTargetPose)
                set_joint_positions(robot, jointNums, originalPoses)
                for i in range(len(movabe)):
                    set_pose(movable[i], originalObjPoses[i])
                yield (cmd,)
    return gen