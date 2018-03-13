from collections import namedtuple
from pybullet_utils import create_box, set_base_values, set_point, set_joint_positions, \
    joint_from_name, get_pose, get_bodies
from pr2_utils import LEFT_JOINT_NAMES, TOP_HOLDING_LEFT_ARM, set_arm_conf, REST_RIGHT_ARM, open_arm, close_arm
import pybullet as p

Problem = namedtuple('Problem', ['robot', 'movable', 'grasp_types', 'surfaces',
                                 'goal_conf', 'goal_holding', 'goal_on'])

def get_fixed_bodies(problem):
    movable = [problem.robot] + list(problem.movable)
    return filter(lambda b: b not in movable, get_bodies())

def holding_problem():
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
                   goal_conf=get_pose(pr2), goal_holding=[('left', box)], goal_on=[])

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
                   goal_conf=get_pose(pr2), goal_holding=[], goal_on=[(box, table)])