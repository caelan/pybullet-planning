#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import time
import argparse

from ss.algorithms.dual_focused import dual_focused
from ss.model.functions import Predicate, rename_functions, initialize, TotalCost
from ss.model.operators import Action, Axiom
from ss.model.plan import print_plan
from ss.model.problem import Problem
from ss.model.streams import GenStream, FnStream

from pr2_utils import get_top_grasps, inverse_kinematics_helper
from pybullet_utils import GraspInfo, link_from_name, WorldSaver
from pybullet_utils import connect, dump_world, get_pose, set_pose, Pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, get_configuration, get_link_pose, \
    SINK_URDF, STOVE_URDF, load_model, wait_for_interrupt, is_placement, sample_placement, wait_for_duration, \
    set_joint_positions, get_body_name, disconnect, \
    get_movable_joints, DRAKE_IIWA_URDF, INF, plan_joint_motion, end_effector_from_body, \
    body_from_end_effector, approach_from_grasp
from pybullet_utils import get_bodies, input

GRASP_INFO = {
    'top': GraspInfo(lambda body: get_top_grasps(body, under=True, tool_pose=Pose(),
                                                 max_width=INF,  grasp_length=0),
                     Pose(0.1*Point(z=1))),
}

TOOL_FRAMES = {
    'iiwa14': 'iiwa_link_ee_kuka', # iiwa_link_ee
}

#######################################################

O, O2 = '?o', '?o2'
P, P2 = '?p', '?p2'
G = '?g'
Q, Q2 = '?q', '?q2'
T = '?t'

IsMovable = Predicate([O])
Stackable = Predicate([O, O2])
Cleaned = Predicate([O])
Cooked = Predicate([O])
Sink = Predicate([O])
Stove = Predicate([O])

IsPose = Predicate([O, P])
IsGrasp = Predicate([O, G])
IsConf = Predicate([Q])
IsTraj = Predicate([T])

IsSupported = Predicate([P, O2])
IsKin = Predicate([O, P, G, Q, T])
IsFreeMotion = Predicate([Q, Q2, T])
IsHoldingMotion = Predicate([Q, Q2, O, G, T])

AtPose = Predicate([O, P])
AtConf = Predicate([Q])
HandEmpty = Predicate([])
HasGrasp = Predicate([O, G])
CanMove = Predicate([])

Holding = Predicate([O])
On = Predicate([O, O2])
Unsafe = Predicate([T])

rename_functions(locals())

#######################################################

class BodyPose(object):
    def __init__(self, body, pose):
        self.body = body
        self.pose = pose
    def assign(self):
        set_pose(self.body, self.pose)
    def __repr__(self):
        return 'p{}'.format(id(self) % 1000)

class BodyGrasp(object):
    def __init__(self, body, grasp_pose, approach_pose, robot, link):
        self.body = body
        self.grasp_pose = grasp_pose
        self.approach_pose = approach_pose
        self.robot = robot
        self.link = link
    def assign(self):
        end_effector_pose = get_link_pose(self.robot, self.link)
        body_pose = body_from_end_effector(end_effector_pose, self.grasp_pose)
        set_pose(self.body, body_pose)
    def __repr__(self):
        return 'g{}'.format(id(self) % 1000)

class BodyConf(object):
    def __init__(self, body, configuration, joints=None):
        if joints is None:
            joints = get_movable_joints(body)
        self.body = body
        self.joints = joints
        self.configuration = configuration
    def assign(self):
        set_joint_positions(self.body, self.joints, self.configuration)
    def __repr__(self):
        return 'q{}'.format(id(self) % 1000)

class BodyPath(object):
    def __init__(self, body, path, joints=None, grasps=[]):
        if joints is None:
            joints = get_movable_joints(body)
        self.body = body
        self.path = path
        self.joints = joints
        self.grasps = grasps
    def bodies(self):
        return set([self.body] + [grasp.body for grasp in self.grasps])
    def iterator(self):
        for i, configuration in enumerate(self.path):
            set_joint_positions(self.body, self.joints, configuration)
            for grasp in self.grasps:
                grasp.assign()
            yield i
    # def full_path(self, q0=None):
    #     # TODO: could produce savers
    #     if q0 is None:
    #         q0 = Conf(self.tree)
    #     new_path = []
    #     for values in self.sequence:
    #         q = q0.copy()
    #         q[self.positions] = values
    #         if self.holding:  # TODO: cache this
    #             kin_cache = self.tree.doKinematics(q)
    #             for body_id, grasp_pose, model_id in self.holding:
    #                 body_pose = get_world_pose(self.tree, kin_cache, body_id)
    #                 model_pose = object_from_gripper(body_pose, grasp_pose)
    #                 set_pose(self.tree, q, model_id, model_pose)
    #         new_path.append(q)
    #     return new_path
    #def refine(self, num_steps=0):
    #    refine_fn = get_refine_fn(self.positions, num_steps)
    #    new_sequence = []
    #    for v1, v2 in zip(self.sequence, self.sequence[1:]):
    #        new_sequence += list(refine_fn(v1, v2))
    #    return self.__class__(self.tree, self.positions, new_sequence, self.holding)
    def reverse(self):
        return self.__class__(self.body, self.path[::-1], self.joints, self.grasps)
    def __repr__(self):
        return 't{}'.format(id(self) % 1000)


class Command(object):
    def __init__(self, body_paths):
        self.body_paths = body_paths

    # def full_path(self, q0=None):
    #     if q0 is None:
    #         q0 = Conf(self.tree)
    #     new_path = [q0]
    #     for partial_path in self.body_paths:
    #         new_path += partial_path.full_path(new_path[-1])[1:]
    #     return new_path

    def step(self):
        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():
                msg = '{},{}) step?'.format(i, j)
                #input(msg)
                print(msg)
                wait_for_interrupt()

    def execute(self, time_step=0.05):
        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():
                #time.sleep(time_step)
                wait_for_duration(time_step)

    def refine(self, num_steps=0):
        return self.__class__([partial_path.refine(num_steps) for partial_path in self.body_paths])

    def reverse(self):
        return self.__class__([partial_path.reverse() for partial_path in reversed(self.body_paths)])

    def __repr__(self):
        return 'c{}'.format(id(self) % 1000)


#######################################################

def get_grasp_gen(robot, grasp_name):
    grasp_info = GRASP_INFO[grasp_name]
    end_effector_link = link_from_name(robot, TOOL_FRAMES[get_body_name(robot)])
    def gen(body):
        grasp_poses = grasp_info.get_grasps(body)
        for grasp_pose in grasp_poses:
            body_grasp = BodyGrasp(body, grasp_pose, grasp_info.approach_pose,
                                   robot, end_effector_link)
            yield (body_grasp,)
    return gen


def get_stable_gen(fixed=[]):
    def gen(body, surface):
        while True:
            pose = sample_placement(body, surface)
            if pose is None:
                continue
            body_pose = BodyPose(body, pose)
            yield (body_pose,)
            # TODO: check collisions
    return gen


def get_ik_fn(robot, fixed=[], teleport=True):
    model_ids = ([robot] + fixed)
    #gripper_id = get_body_from_name(tree, KUKA_TOOL_FRAME, robot).get_body_index()
    def fn(model, pose, grasp):
        gripper_pose = end_effector_from_body(pose.pose, grasp.grasp_pose)
        approach_pose = approach_from_grasp(grasp.approach_pose, gripper_pose)


        q_approach = inverse_kinematics_helper(robot, grasp.link, approach_pose)
        print(q_approach)
        if (q_approach is None): # or are_colliding(tree, tree.doKinematics(q_approach), model_ids=model_ids):
            return None
        conf = BodyConf(robot, q_approach)
        q_grasp = inverse_kinematics_helper(robot, grasp.link, gripper_pose)
        if (q_grasp is None): # or are_colliding(tree, tree.doKinematics(q_grasp), model_ids=model_ids):
            return None
        if teleport:
            path = [q_approach, q_grasp]
        else:
            conf.assign()
            path = plan_joint_motion(robot, conf.joints, q_grasp, obstacles=[], direct=True)
            if path is None:
                raw_input('Approach motion failed')
                return None
        command = Command([BodyPath(robot, path),
                           BodyPath(robot, path[::-1], grasps=[grasp])])
        return (conf, command)
        # TODO: holding collisions
    return fn


def get_motion_gen(robot, fixed=[], teleport=True):
    model_ids = ([robot] + fixed)
    def fn(conf1, conf2):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            conf1.assign()
            path = plan_joint_motion(robot, conf2.joints, conf2.configuration, obstacles=[])
            if path is None:
                raw_input('Free motion failed')
                return None
        command = Command([BodyPath(robot, path)])
        return (command,)
    return fn


def get_holding_motion_gen(robot, fixed=[], teleport=True):
    model_ids = ([robot] + fixed)
    def fn(conf1, conf2, body, grasp):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            # TODO: holding
            conf1.assign()
            path = plan_joint_motion(robot, conf2.joints, conf2.configuration, obstacles=[])
            if path is None:
                raw_input('Holding motion failed')
                return None
        command = Command([BodyPath(robot, path, grasps=[grasp])])
        return (command,)
    return fn


def get_movable_collision_test():
    def test(command, model, pose):
        for partial_path in command.partial_paths:
            if any(info.model_id == model for info in partial_path.holding):
                continue  # Cannot collide with itself
            # TODO: cache the KinematicsCaches
            q = Conf(tree)
            pose.assign(q)  # TODO: compute kinematics trees just for pairs/triplets of objects
            model_ids = partial_path.model_ids() + [model]
            for q in command.full_path(q):
                if are_colliding(tree, tree.doKinematics(q), model_ids=model_ids):
                    raw_input('Movable collision')
                    return True
        return False
    return test


#######################################################

def ss_from_problem(robot, movable=[], bound='shared', teleport=False, movable_collisions=False, grasp_name='top'):
    #assert (not are_colliding(tree, kin_cache))
    rigid = [body for body in get_bodies() if body != robot]
    fixed = [body for body in rigid if body not in movable]
    print('Robot:', robot)
    print('Movable:', movable)
    print('Fixed:', fixed)

    conf = BodyConf(robot, get_configuration(robot))
    initial_atoms = [
        HandEmpty(), CanMove(),
        IsConf(conf), AtConf(conf),
        initialize(TotalCost(), 0),
    ]
    for body in movable:
        pose = BodyPose(body, get_pose(body))
        initial_atoms += [IsMovable(body), IsPose(body, pose), AtPose(body, pose)]
        for surface in fixed:
            initial_atoms += [Stackable(body, surface)]
            if is_placement(body, surface):
                initial_atoms += [IsSupported(pose, surface)]

    for body in fixed:
        name = get_body_name(body)
        if 'sink' in name:
            initial_atoms.append(Sink(body))
        if 'stove' in name:
            initial_atoms.append(Stove(body))

    body = movable[0]
    goal_literals = [
        AtConf(conf),
        Holding(body),
        #On(body, fixed[0]),
        #On(body, fixed[2]),
        # Cleaned(body),
        #Cooked(body),
    ]

    PlacedCollision = Predicate([T, O, P], domain=[IsTraj(T), IsPose(O, P)],
                                fn=get_movable_collision_test(),
                                bound=False)

    actions = [
        Action(name='pick', param=[O, P, G, Q, T],
               pre=[IsKin(O, P, G, Q, T),
                    HandEmpty(), AtPose(O, P), AtConf(Q), ~Unsafe(T)],
               eff=[HasGrasp(O, G), CanMove(), ~HandEmpty(), ~AtPose(O, P)]),

        Action(name='place', param=[O, P, G, Q, T],
               pre=[IsKin(O, P, G, Q, T),
                    HasGrasp(O, G), AtConf(Q), ~Unsafe(T)],
               eff=[HandEmpty(), CanMove(), AtPose(O, P), ~HasGrasp(O, G)]),

        # Can just do move if we check holding collisions
        Action(name='move_free', param=[Q, Q2, T],
               pre=[IsFreeMotion(Q, Q2, T),
                    HandEmpty(), CanMove(), AtConf(Q), ~Unsafe(T)],
               eff=[AtConf(Q2), ~CanMove(), ~AtConf(Q)]),

        Action(name='move_holding', param=[Q, Q2, O, G, T],
               pre=[IsHoldingMotion(Q, Q2, O, G, T),
                    HasGrasp(O, G), CanMove(), AtConf(Q), ~Unsafe(T)],
               eff=[AtConf(Q2), ~CanMove(), ~AtConf(Q)]),

        Action(name='clean', param=[O, O2],  # Wirelessly communicates to clean
               pre=[Stackable(O, O2), Sink(O2),
                    ~Cooked(O), On(O, O2)],
               eff=[Cleaned(O)]),

        Action(name='cook', param=[O, O2],  # Wirelessly communicates to cook
               pre=[Stackable(O, O2), Stove(O2),
                    Cleaned(O), On(O, O2)],
               eff=[Cooked(O), ~Cleaned(O)]),
    ]

    axioms = [
        Axiom(param=[O, G],
              pre=[IsGrasp(O, G),
                   HasGrasp(O, G)],
              eff=Holding(O)),
        Axiom(param=[O, P, O2],
              pre=[IsPose(O, P), IsSupported(P, O2),
                   AtPose(O, P)],
              eff=On(O, O2)),
    ]
    if movable_collisions:
        axioms += [
            Axiom(param=[T, O, P],
                  pre=[IsPose(O, P), PlacedCollision(T, O, P),
                       AtPose(O, P)],
                  eff=Unsafe(T)),
        ]

    streams = [
        GenStream(name='grasp', inp=[O], domain=[IsMovable(O)],
                  fn=get_grasp_gen(robot, grasp_name), out=[G],
                  graph=[IsGrasp(O, G)], bound=bound),

        # TODO: test_support
        GenStream(name='support', inp=[O, O2], domain=[Stackable(O, O2)],
                  fn=get_stable_gen(fixed=fixed), out=[P],
                  graph=[IsPose(O, P), IsSupported(P, O2)], bound=bound),

        FnStream(name='inverse_kin', inp=[O, P, G], domain=[IsPose(O, P), IsGrasp(O, G)],
                 fn=get_ik_fn(robot, fixed=fixed, teleport=teleport), out=[Q, T],
                 graph=[IsKin(O, P, G, Q, T), IsConf(Q), IsTraj(T)], bound=bound),

        FnStream(name='free_motion', inp=[Q, Q2], domain=[IsConf(Q), IsConf(Q2)],
                 fn=get_motion_gen(robot, teleport=teleport), out=[T],
                 graph=[IsFreeMotion(Q, Q2, T), IsTraj(T)], bound=bound),

        FnStream(name='holding_motion', inp=[Q, Q2, O, G], domain=[IsConf(Q), IsConf(Q2), IsGrasp(O, G)],
                 fn=get_holding_motion_gen(robot, teleport=teleport), out=[T],
                 graph=[IsHoldingMotion(Q, Q2, O, G, T), IsTraj(T)], bound=bound),
    ]

    return Problem(initial_atoms, goal_literals, actions, axioms, streams,
                   objective=TotalCost())

#######################################################

def load_world():
    #print(get_data_path())
    #p.loadURDF("samurai.urdf", useFixedBase=True) # World
    #p.loadURDF("kuka_lwr/kuka.urdf", useFixedBase=True)
    #p.loadURDF("kuka_iiwa/model_free_base.urdf", useFixedBase=True)

    # TODO: store internal world info here to be reloaded
    robot = load_model(DRAKE_IIWA_URDF)
    # robot = load_model(KUKA_IIWA_URDF)
    floor = load_model('models/short_floor.urdf')
    sink = load_model(SINK_URDF, pose=Pose(Point(x=-0.5)))
    stove = load_model(STOVE_URDF, pose=Pose(Point(x=+0.5)))
    block = load_model(BLOCK_URDF)

    set_pose(block, Pose(Point(y=0.5, z=stable_z(block, floor))))
    # print(get_camera())
    set_default_camera()

    return robot, block

def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    parser.add_argument('-display', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    connect(use_gui=args.viewer)
    robot, block = load_world()
    saved_world = WorldSaver()
    dump_world()

    ss_problem = ss_from_problem(robot, movable=[block], teleport=False, movable_collisions=False)
    #ss_problem = ss_problem.debug_problem()
    #print(ss_problem)

    t0 = time.time()
    plan, evaluations = dual_focused(ss_problem, verbose=True)
    # plan, evaluations = incremental(ss_problem, verbose=True)
    print_plan(plan, evaluations)
    print(time.time() - t0)
    if (not args.display) or (plan is None):
        p.disconnect()
        return

    paths = []
    for action, params in plan:
        if action.name == 'place':
            paths += params[-1].reverse().body_paths
        elif action.name in ['move_free', 'move_holding', 'pick']:
            paths += params[-1].body_paths
    print(paths)
    command = Command(paths)

    if not args.viewer: # TODO: how to reenable the viewer
        disconnect()
        connect(use_gui=True)
        load_world()
    saved_world.restore()

    input('Execute?')
    command.execute(time_step=0.05)
    #command.step()

    wait_for_interrupt()
    disconnect()

if __name__ == '__main__':
    main()