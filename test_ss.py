import pybullet as p
import os
import argparse
import time
import pstats
import cProfile

from pybullet_utils import connect, add_data_path, disconnect, get_pose, get_body_names, body_from_name, update_state
from problems import holding_problem

from ss.algorithms.dual_focused import dual_focused
from ss.algorithms.incremental import incremental
from ss.algorithms.plan_focused import plan_focused
from ss.model.functions import Predicate, NonNegFunction, rename_functions, initialize, TotalCost, Increase
from ss.model.problem import Problem, get_length, get_cost
from ss.model.operators import Action, Axiom
from ss.model.streams import Stream, ListStream, GenStream, FnStream, TestStream
from ss.model.bounds import PartialBoundFn, OutputSet
from ss.utils import INF

from streams import Pose, Conf, get_ik_ir_gen, get_motion_gen, get_stable_gen, get_grasp_gen

A = '?a'
O = '?o'; O2 = '?o2'
#S = '?s' # Surface (in case I don't want to worry about stacking for real
P = '?p'; P2 = '?p2'
G = '?g'
BQ = '?q'; BQ2 = '?q2'
AT = '?at'; BT = '?bt'

IsArm = Predicate([A])
IsMovable = Predicate([O])
Stackable = Predicate([O, O2])

POSE = Predicate([P])
GRASP = Predicate([G])
IsBConf = Predicate([BQ])

IsPose = Predicate([O, P])
IsSupported = Predicate([P, O2])
IsGrasp = Predicate([O, G])
IsArmTraj = Predicate([AT])
IsBaseTraj = Predicate([BT])
IsKin = Predicate([A, O, P, G, BQ, AT])
IsReachable = Predicate([BQ])
IsMotion = Predicate([BQ, BQ2, BT])

AtPose = Predicate([O, P])
AtBConf = Predicate([BQ])
HandEmpty = Predicate([A])
HasGrasp = Predicate([A, O, G])
CanMove = Predicate([])

# Derived
Holding = Predicate([A, O])
On = Predicate([O, O2])
Unsafe = Predicate([AT])

rename_functions(locals())


def ss_from_problem(problem, bound='cyclic'):
    robot = problem.robot
    arms = ['left']
    #arms = ['left', 'right']

    initial_bq = Pose(robot, get_pose(robot))
    initial_atoms = [
        CanMove(),
        IsBConf(initial_bq), AtBConf(initial_bq),
        initialize(TotalCost(), 0),
    ]
    for name in arms:
        initial_atoms += [IsArm(name), HandEmpty(name)]
    for body in problem.movable:
        pose = Pose(body, get_pose(body))
        initial_atoms += [IsMovable(body), IsPose(body, pose), AtPose(body, pose), POSE(pose)]
        for surface in problem.surfaces:
            initial_atoms += [Stackable(body, surface)]

    goal_literals = []
    goal_literals += [On(*pair) for pair in problem.goal_on]
    if problem.goal_conf is not None:
        goal_conf = Pose(robot, problem.goal_conf)
        initial_atoms += [IsBConf(goal_conf)]
        goal_literals += [AtBConf(goal_conf)]

    actions = [
        Action(name='pick', param=[A, O, P, G, BQ, AT],
               pre=[IsKin(A, O, P, G, BQ, AT),
                    HandEmpty(A), AtPose(O, P), AtBConf(BQ), ~Unsafe(AT)],
               eff=[HasGrasp(A, O, G), CanMove(), ~HandEmpty(A), ~AtPose(O, P),
                    Increase(TotalCost(), 1)]),

        Action(name='place', param=[A, O, P, G, BQ, AT],
               pre=[IsKin(A, O, P, G, BQ, AT),
                    HasGrasp(A, O, G), AtBConf(BQ), ~Unsafe(AT)],
               eff=[HandEmpty(A), CanMove(), AtPose(O, P), ~HasGrasp(A, O, G),
                    Increase(TotalCost(), 1)]),

        Action(name='move', param=[BQ, BQ2, BT],
               pre=[IsMotion(BQ, BQ2, BT),
                    CanMove(), AtBConf(BQ), ~Unsafe(BT)],
               eff=[AtBConf(BQ2), ~CanMove(), ~AtBConf(BQ),
                    Increase(TotalCost(), 1)]),
    ]
    axioms = [
        Axiom(param=[A, O, G],
              pre=[IsArm(A), IsGrasp(O, G),
                   HasGrasp(A, O, G)],
              eff=Holding(A, O)),
        Axiom(param=[O, P, O2],
              pre=[IsPose(O, P), IsSupported(P, O2),
                   AtPose(O, P)],
              eff=On(O, O2)),
    ]

    streams = [
        FnStream(name='motion', inp=[BQ, BQ2], domain=[IsBConf(BQ), IsBConf(BQ2)],
                 fn=get_motion_gen(problem), out=[BT],
                 graph=[IsMotion(BQ, BQ2, BT), IsBaseTraj(BT)], bound=bound),

        ListStream(name='grasp', inp=[O], domain=[IsMovable(O)], fn=get_grasp_gen(problem),
                   out=[G], graph=[IsGrasp(O, G), GRASP(G)], bound=bound),

        Stream(name='support', inp=[O, O2], domain=[Stackable(O, O2)],
               fn=get_stable_gen(problem), out=[P],
               graph=[IsPose(O, P), IsSupported(P, O2), POSE(P)], bound=bound),

        GenStream(name='ik_ir', inp=[A, O, P, G], domain=[IsArm(A), IsPose(O, P), IsGrasp(O, G)],
                  fn=get_ik_ir_gen(problem), out=[BQ, AT],
                  graph=[IsKin(A, O, P, G, BQ, AT), IsBConf(BQ), IsArmTraj(AT)],
                  bound=bound),
    ]

    return Problem(initial_atoms, goal_literals, actions, axioms, streams,
                   objective=TotalCost())

def post_process(problem, plan):
    if plan is None:
        return None
    commands = []
    for i, (action, args) in enumerate(plan):
        print i, action, args
        if action.name == 'move':
            traj = args[-1]
            new_commands = [traj]
        elif action.name == 'pick':
            traj = args[-1]
            new_commands = [traj]
        elif action.name == 'place':
            traj = args[-1].reverse()
            new_commands = [traj]
        else:
            raise ValueError(action.name)
        commands += new_commands
    return commands

def main(search='ff-astar', max_time=30, verbose=False):
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()
    problem_fn = holding_problem

    print connect(use_gui=False)
    add_data_path()

    problem = problem_fn()
    print get_body_names()
    pr2 = body_from_name('pr2')
    print get_pose(pr2)

    ss_problem = ss_from_problem(problem)
    ss_problem.dump()

    #path = os.path.join('worlds', 'test_ss')
    #p.saveWorld(path)
    #state_id = p.saveState()
    #p.saveBullet(path)

    t0 = time.time()
    pr = cProfile.Profile()
    pr.enable()
    #plan, evaluations = incremental(ss_problem, planner=search, max_time=max_time,
    #                                verbose=verbose, terminate_cost=terminate_cost)
    plan, evaluations = dual_focused(ss_problem, planner=search, max_time=max_time,
                                     effort_weight=None, verbose=verbose)
    pr.disable()
    pstats.Stats(pr).sort_stats('tottime').print_stats(10) # tottime | cumtime

    print 'Plan:', plan
    print 'Solved:', plan is not None
    print 'Length:', get_length(plan, evaluations)
    print 'Cost:', get_cost(plan, evaluations)
    print 'Time:', time.time() - t0
    if (plan is None) or not args.viewer:
        return

    #state_id = p.saveState()
    #disconnect()
    #connect(use_gui=args.viewer)
    #p.restoreState(state_id)

    disconnect()
    connect(use_gui=args.viewer)
    problem = problem_fn() # TODO: way of doing this without reloading?
    update_state()
    raw_input('Begin?')
    attachments = {}
    for i, (action, args) in enumerate(plan):
        print i, action, args
        control = args[-1]
        if action.name == 'place':
            control = control.reverse()
        control.step()

    #p.stepSimulation()
    raw_input('Finish?')
    disconnect()


if __name__ == '__main__':
    main()