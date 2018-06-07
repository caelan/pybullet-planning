#!/usr/bin/env python

from __future__ import print_function

import argparse
import time

from ss.algorithms.dual_focused import dual_focused
from ss.model.functions import Predicate, rename_functions, initialize, TotalCost
from ss.model.operators import Action, Axiom
from ss.model.plan import print_plan
from ss.model.problem import Problem
from ss.model.streams import GenStream, FnStream

from pybullet_tools.kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, get_stable_gen, get_ik_fn, get_free_motion_gen, \
    get_holding_motion_gen, get_movable_collision_test
from pybullet_tools.utils import WorldSaver,  connect, dump_world, get_pose, set_pose, Pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, get_configuration, SINK_URDF, STOVE_URDF, load_model, wait_for_interrupt, is_placement, get_body_name, \
    disconnect, DRAKE_IIWA_URDF, get_bodies, user_input

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

def ss_from_problem(robot, movable=[], bound='shared',
                    teleport=False, movable_collisions=False, grasp_name='top'):
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
        #Holding(body),
        #On(body, fixed[0]),
        #On(body, fixed[2]),
        #Cleaned(body),
        Cooked(body),
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
                 fn=get_free_motion_gen(robot, teleport=teleport), out=[T],
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
    block = load_model(BLOCK_URDF, fixed_base=False)
    #cup = load_model('models/dinnerware/cup/cup_small.urdf',
    # Pose(Point(x=+0.5, y=+0.5, z=0.5)), fixed_base=False)

    set_pose(block, Pose(Point(y=0.5, z=stable_z(block, floor))))
    # print(get_camera())
    set_default_camera()

    return robot, block

def main(execute='execute'):
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    #parser.add_argument('-display', action='store_true', help='enable viewer.')
    args = parser.parse_args()
    #display = args.display
    display = True

    connect(use_gui=args.viewer)
    robot, block = load_world()

    #robot2 = clone_body(robot)
    #block2 = clone_body(block)
    #dump_world()

    saved_world = WorldSaver()
    dump_world()

    ss_problem = ss_from_problem(robot, movable=[block], teleport=False, movable_collisions=True)
    #ss_problem = ss_problem.debug_problem()
    #print(ss_problem)

    t0 = time.time()
    plan, evaluations = dual_focused(ss_problem, verbose=True)
    # plan, evaluations = incremental(ss_problem, verbose=True)
    print_plan(plan, evaluations)
    print(time.time() - t0)
    if (not display) or (plan is None):
        disconnect()
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

    user_input('Execute?')
    if execute == 'control':
        command.control()
    elif execute == 'execute':
        command.refine(num_steps=10).execute(time_step=0.001)
    elif execute == 'step':
        command.step()
    else:
        raise ValueError(execute)

    #dt = 1. / 240 # Bullet default
    #p.setTimeStep(dt)

    wait_for_interrupt()
    disconnect()

if __name__ == '__main__':
    main()