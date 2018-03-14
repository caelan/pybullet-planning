from collections import namedtuple
from pybullet_utils import create_box, set_base_values, set_point, set_pose, get_pose, get_bodies, z_rotation
from pr2_utils import TOP_HOLDING_LEFT_ARM, set_arm_conf, REST_LEFT_ARM, REST_RIGHT_ARM, open_arm, \
    close_arm, get_carry_conf, arm_conf, get_other_arm
import pybullet as p

#Problem = namedtuple('Problem', ['robot', 'arms', 'movable', 'grasp_types', 'surfaces',
#                                 'goal_conf', 'goal_holding', 'goal_on'])

class Problem(object):
    def __init__(self, robot, arms=tuple(), movable=tuple(), grasp_types=tuple(),
                 surfaces=tuple(), sinks=tuple(), stoves=tuple(), buttons=tuple(),
                 goal_conf=None, goal_holding=tuple(), goal_on=tuple(),
                 goal_cleaned=tuple(), goal_cooked=tuple()):
        self.robot = robot
        self.arms = arms
        self.movable = movable
        self.grasp_types = grasp_types
        self.surfaces = surfaces
        self.sinks = sinks
        self.stoves = stoves
        self.buttons = buttons
        self.goal_conf = goal_conf
        self.goal_holding = goal_holding
        self.goal_on = goal_on
        self.goal_cleaned = goal_cleaned
        self.goal_cooked = goal_cooked


def get_fixed_bodies(problem):
    movable = [problem.robot] + list(problem.movable)
    return filter(lambda b: b not in movable, get_bodies())

def holding_problem(arm='left', grasp_type='side'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    #pr2 = p.loadURDF("pr2_description/pr2.urdf", useFixedBase=True)
    pr2 = p.loadURDF("pr2_description/pr2_fixed_torso.urdf", useFixedBase=True)
    set_base_values(pr2, (0, -2, 0))
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    plane = p.loadURDF("plane.urdf")
    table = p.loadURDF("table/table.urdf")
    #table = p.loadURDF("table_square/table_square.urdf")
    box = create_box(.07, .05, .15)
    set_point(box, (0, 0, .7))

    return Problem(robot=pr2, movable=[box], arms=[arm], grasp_types=[grasp_type], surfaces=[table],
                   goal_conf=get_pose(pr2), goal_holding=[(arm, box)])

def stacking_problem():
    #pr2 = p.loadURDF("pr2_description/pr2.urdf", useFixedBase=True)
    pr2 = p.loadURDF("pr2_description/pr2_fixed_torso.urdf", useFixedBase=True)
    set_base_values(pr2, (0, -2, 0))
    set_arm_conf(pr2, 'left', TOP_HOLDING_LEFT_ARM)
    open_arm(pr2, 'left')
    set_arm_conf(pr2, 'right', REST_RIGHT_ARM)
    close_arm(pr2, 'right')

    plane = p.loadURDF("plane.urdf")
    table = p.loadURDF("table/table.urdf")
    #table = p.loadURDF("table_square/table_square.urdf")
    box = create_box(.07, .05, .15)
    set_point(box, (0, 0, .7))

    return Problem(robot=pr2, movable=[box], grasp_types=['top'], surfaces=[table],
                   goal_on=[(box, table)])

def create_kitchen(w=.5, h=.7):
    plane = p.loadURDF("plane.urdf")

    table = create_box(w, w, h, color=(.75, .75, .75, 1))
    set_point(table, (2, 0, h/2))

    cabbage = create_box(.07, .07, .1, color=(0, 1, 0, 1))
    set_point(cabbage, (2, 0, h + .1/2))

    sink = create_box(w, w, h, color=(.25, .25, .75, 1))
    set_point(sink, (0, 2, h/2))

    stove = create_box(w, w, h, color=(.75, .25, .25, 1))
    set_point(stove, (0, -2, h/2))

    return table, cabbage, sink, stove

def cleaning_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = p.loadURDF("pr2_description/pr2_fixed_torso.urdf", useFixedBase=True)
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
                   goal_cleaned=[cabbage])

def cooking_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = p.loadURDF("pr2_description/pr2_fixed_torso.urdf", useFixedBase=True)
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
                   goal_cooked=[cabbage])

def cleaning_button_problem(arm='left', grasp_type='top'):
    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    pr2 = p.loadURDF("pr2_description/pr2_fixed_torso.urdf", useFixedBase=True)
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)

    table, cabbage, sink, stove = create_kitchen()

    d = 0.1
    sink_button = create_box(d, d, d, color=(0, 0, 0, 1))
    set_point(sink_button, (0, 2-(.5+d)/2, .7-d/2))

    stove_button = create_box(d, d, d, color=(0, 0, 0, 1))
    set_point(stove_button, (0, -2+(.5+d)/2, .7-d/2))

    return Problem(robot=pr2, movable=[cabbage], arms=[arm], grasp_types=[grasp_type],
                   surfaces=[table, sink, stove], sinks=[sink], stoves=[stove],
                   buttons=[(sink_button, sink), (stove_button, stove)],
                   goal_cleaned=[cabbage])