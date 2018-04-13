import argparse
import time
import pstats
import cProfile

from utils import connect, add_data_path, disconnect, get_pose, \
    update_state, link_from_name, step_simulation, is_placement, joints_from_names, get_joint_positions, input
from pr2_problems import holding_problem, stacking_problem, cleaning_problem, cooking_problem, \
    cleaning_button_problem, cooking_button_problem

from ss.algorithms.dual_focused import dual_focused
from ss.algorithms.incremental import incremental
from ss.model.functions import Predicate, NonNegFunction, rename_functions, initialize, TotalCost, Increase
from ss.model.problem import Problem, get_length, get_cost
from ss.model.operators import Action, Axiom
from ss.model.streams import Stream, ListStream, GenStream, FnStream, TestStream
from ss.model.plan import print_plan

from pr2_primitives import Pose, Conf, get_ik_ir_gen, get_motion_gen, get_stable_gen, \
    get_grasp_gen, get_press_gen, Attach, Detach, Clean, Cook, Trajectory
from pr2_utils import ARM_LINK_NAMES, close_arm, ARM_JOINT_NAMES, get_arm_joints

import pybullet as p

A = '?a'
O = '?o'; O2 = '?o2'
B = '?b'
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
IsAConf = Predicate([A, BQ])

IsPose = Predicate([O, P])
IsSupported = Predicate([P, O2])
IsGrasp = Predicate([O, G])
IsArmTraj = Predicate([AT])
IsBaseTraj = Predicate([BT])
IsKin = Predicate([A, O, P, G, BQ, AT])
IsMotion = Predicate([BQ, BQ2, BT])

AtPose = Predicate([O, P])
AtBConf = Predicate([BQ])
HandEmpty = Predicate([A])
HasGrasp = Predicate([A, O, G])
CanMove = Predicate([])
AtAConf = Predicate([A, BQ])

# Derived
Holding = Predicate([A, O])
On = Predicate([O, O2])
Unsafe = Predicate([AT])

Cleaned = Predicate([O])
Cooked = Predicate([O])
Washer = Predicate([O])
Stove = Predicate([O])

# TODO: can either make a single button for thing or can connect
IsButton = Predicate([O])
IsPress = Predicate([A, B, BQ, AT])
IsConnected = Predicate([O, O2])
#Pressed = Predicate([O])

rename_functions(locals())


def ss_from_problem(problem, bound='shared', remote=False, teleport=False, movable_collisions=False):
    robot = problem.robot

    initial_bq = Pose(robot, get_pose(robot))
    initial_atoms = [
        CanMove(),
        IsBConf(initial_bq), AtBConf(initial_bq),
        initialize(TotalCost(), 0),
    ]
    for name in problem.arms:
        joints = get_arm_joints(robot, name)
        conf = Conf(robot, joints, get_joint_positions(robot, joints))
        initial_atoms += [IsArm(name), HandEmpty(name), IsAConf(name, conf), AtAConf(name, conf)]
    for body in problem.movable:
        pose = Pose(body, get_pose(body))
        initial_atoms += [IsMovable(body), IsPose(body, pose), AtPose(body, pose), POSE(pose)]
        for surface in problem.surfaces:
            initial_atoms += [Stackable(body, surface)]
            if is_placement(body, surface):
                initial_atoms += [IsSupported(pose, surface)]

    initial_atoms += map(Washer, problem.sinks)
    initial_atoms += map(Stove, problem.stoves)
    initial_atoms += [IsConnected(*pair) for pair in problem.buttons]
    initial_atoms += [IsButton(body) for body, _ in problem.buttons]

    goal_literals = []
    if problem.goal_conf is not None:
        goal_conf = Pose(robot, problem.goal_conf)
        initial_atoms += [IsBConf(goal_conf)]
        goal_literals += [AtBConf(goal_conf)]
    goal_literals += [Holding(*pair) for pair in problem.goal_holding]
    goal_literals += [On(*pair) for pair in problem.goal_on]
    goal_literals += map(Cleaned, problem.goal_cleaned)
    goal_literals += map(Cooked, problem.goal_cooked)

    #GraspedCollision = Predicate([A, G, AT], domain=[IsArm(A), POSE(G), IsArmTraj(AT)],
    #                             fn=lambda a, p, at: False, bound=False)

    PlacedCollision = Predicate([AT, P], domain=[IsArmTraj(AT), POSE(P)],
                                fn=lambda at, p: False,
                                bound=False)

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

        # TODO: should the robot be allowed to have different carry configs or a defined one?
        #Action(name='move_arm', param=[A, BQ, BQ2],
        #       pre=[IsAConf(A, BQ), IsAConf(A, BQ),
        #            CanMove(), AtBConf(BQ)],
        #       eff=[AtAConf(BQ2), ~AtAConf(BQ),
        #            Increase(TotalCost(), 1)]),

        # Why would one want to move to the pick configuration for something anywhere but the goal?
        # Stream that generates arm motions as long as one is a shared configuration
        # Maybe I should make a base
    ]

    if remote:
        actions += [
            Action(name='clean', param=[O, O2],  # Wirelessly communicates to clean
                  pre=[Stackable(O, O2), Washer(O2),
                       ~Cooked(O), On(O, O2)],
                  eff=[Cleaned(O)]),

            Action(name='cook', param=[O, O2],  # Wirelessly communicates to cook
                  pre=[Stackable(O, O2), Stove(O2),
                       Cleaned(O), On(O, O2)],
                  eff=[Cooked(O), ~Cleaned(O)]),
        ]
    else:
        actions += [
            Action(name='press_clean', param=[O, O2, A, B, BQ, AT],
                   pre=[Stackable(O, O2), Washer(O2), IsConnected(B, O2), IsPress(A, B, BQ, AT),
                        ~Cooked(O), On(O, O2), HandEmpty(A), AtBConf(BQ)],
                   eff=[Cleaned(O), CanMove()]),

            Action(name='press_cook', param=[O, O2, A, B, BQ, AT],
                   pre=[Stackable(O, O2), Stove(O2), IsConnected(B, O2), IsPress(A, B, BQ, AT),
                        Cleaned(O), On(O, O2), HandEmpty(A), AtBConf(BQ)],
                   eff=[Cooked(O), ~Cleaned(O), CanMove()]),
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
    if movable_collisions:
        axioms += [
            Axiom(param=[O, P, AT],
                  pre=[IsPose(O, P), PlacedCollision(AT, P),
                       AtPose(O, P)],
                  eff=Unsafe(AT)),
        ]

    streams = [
        FnStream(name='motion', inp=[BQ, BQ2], domain=[IsBConf(BQ), IsBConf(BQ2)],
                 fn=get_motion_gen(problem, teleport=teleport), out=[BT],
                 graph=[IsMotion(BQ, BQ2, BT), IsBaseTraj(BT)], bound=bound),

        ListStream(name='grasp', inp=[O], domain=[IsMovable(O)], fn=get_grasp_gen(problem),
                   out=[G], graph=[IsGrasp(O, G), GRASP(G)], bound=bound),

        # TODO: test_support
        Stream(name='support', inp=[O, O2], domain=[Stackable(O, O2)],
               fn=get_stable_gen(problem), out=[P],
               graph=[IsPose(O, P), IsSupported(P, O2), POSE(P)], bound=bound),

        GenStream(name='ik_ir', inp=[A, O, P, G], domain=[IsArm(A), IsPose(O, P), IsGrasp(O, G)],
                  fn=get_ik_ir_gen(problem, teleport=teleport), out=[BQ, AT],
                  graph=[IsKin(A, O, P, G, BQ, AT), IsBConf(BQ), IsArmTraj(AT)],
                  bound=bound),

        GenStream(name='press', inp=[A, B], domain=[IsArm(A), IsButton(B)],
                  fn=get_press_gen(problem, teleport=teleport), out=[BQ, AT],
                  graph=[IsPress(A, B, BQ, AT), IsBConf(BQ), IsArmTraj(AT)],
                  bound=bound),
    ]

    return Problem(initial_atoms, goal_literals, actions, axioms, streams,
                   objective=TotalCost())

def post_process(problem, plan):
    if plan is None:
        return None
    robot = problem.robot
    commands = []
    for i, (action, args) in enumerate(plan):
        print i, action, args
        if action.name == 'move':
            t = args[-1]
            new_commands = [t]
        elif action.name == 'pick':
            a, b, p, g, _, t = args
            #link = link_from_name(robot, ARM_LINK_NAMES[a])
            attach = Attach(robot, a, g, b)
            new_commands = [t, attach, t.reverse()]
        elif action.name == 'place':
            a, b, p, g, _, t = args
            #link = link_from_name(robot, ARM_LINK_NAMES[a])
            detach = Detach(robot, a, b)
            new_commands = [t, detach, t.reverse()]
        elif action.name == 'clean': # TODO: add text or change color?
            body, sink = args
            new_commands = [Clean(body)]
        elif action.name == 'cook':
            body, stove = args
            new_commands = [Cook(body)]
        elif action.name == 'press_clean':
            body, sink, arm, button, bq, t = args
            new_commands = [t, Clean(body), t.reverse()]
        elif action.name == 'press_cook':
            body, sink, arm, button, bq, t = args
            new_commands = [t, Cook(body), t.reverse()]
        else:
            raise ValueError(action.name)
        commands += new_commands
    return commands

def step_commands(commands, time_step=None, simulate=False):
    # update_state()
    if simulate: step_simulation()
    input('Begin?')
    attachments = {}
    for i, command in enumerate(commands):
        print i, command
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
                if simulate: step_simulation()
                if time_step is None:
                    input('Continue?')
                else:
                    time.sleep(time_step)
        elif type(command) in [Clean, Cook]:
            command.step()
        else:
            raise ValueError(command)

def main(search='ff-astar', max_time=60, verbose=True):
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    parser.add_argument('-display', action='store_true', help='enable viewer.')
    args = parser.parse_args()
    problem_fn = holding_problem
    # holding_problem | stacking_problem | cleaning_problem | cooking_problem
    # cleaning_button_problem | cooking_button_problem

    connect(use_gui=args.viewer)
    add_data_path()

    problem = problem_fn()
    state_id = p.saveState()

    ss_problem = ss_from_problem(problem, remote=False, teleport=False)
    print ss_problem
    #ss_problem.dump()

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

    print 'Time:', time.time() - t0
    print_plan(plan, evaluations)
    if (plan is None) or not args.display:
        disconnect()
        return

    if args.viewer:
        p.restoreState(state_id)
    else:
        disconnect()
        connect(use_gui=True)
        problem = problem_fn()  # TODO: way of doing this without reloading?

    commands = post_process(problem, plan)
    #step_commands(commands)
    step_commands(commands, time_step=0.01)
    input('Finish?')
    disconnect()


if __name__ == '__main__':
    main()
