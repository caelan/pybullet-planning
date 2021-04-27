#!/usr/bin/env python

from __future__ import print_function

import json
import os

from pybullet_tools.parse_json import parse_pose, parse_robot, parse_region, parse_body
from pybullet_tools.utils import connect, \
    disconnect, wait_for_interrupt, point_from_pose, set_camera_pose, \
    reset_simulation, wait_if_gui
from pybullet_tools.pr2_problems import Problem

BODY_FIELDS = [u'table_names', u'sink_names', u'stove_names']

# [u'goal_cleaned', u'regions', u'goal_stackings', u'name',
# u'sink_names', u'goal_regions', u'goal_cooked',
# u'goal_config', u'table_names', u'floor_object_names',
# u'grasp_types', u'goal_holding', u'object_names', u'stove_names', u'goal_poses']

def parse_task(task, robots, bodies, regions):
    [robot] = robots.values()
    arm = 'left'

    goal_holding = []
    if task['goal_holding'] is not None:
        goal_holding.append((arm, bodies[task['goal_holding']]))

    # TODO: combine robots, bodies, regions
    arms = [arm]
    movable = [bodies[n] for n in task['object_names']]
    grasp_types = [grasp_type.lower() for grasp_type in task['grasp_types']]
    #grasp_types = ['top']
    sinks = [bodies[n] for n in task['sink_names']]
    stoves = [bodies[n] for n in task['stove_names']]
    surfaces = [bodies[n] for n in task['table_names']] + list(regions.values()) + sinks + stoves
    #buttons = tuple()
    #assert not task['goal_poses']
    assert task['goal_config'] is None
    goal_conf = None
    goal_on = [(bodies[n1], bodies[n2]) for n1, n2 in task['goal_stackings'].items()] + \
              [(bodies[n1], regions[n2]) for n1, n2 in task['goal_regions'].items()]
    goal_cleaned = [bodies[n] for n in task['goal_cleaned']]
    goal_cooked = [bodies[n] for n in task['goal_cooked']]

    body_names = {body: name for name, body in list(bodies.items()) + list(regions.items())}

    return Problem(robot=robot, arms=arms, movable=movable, grasp_types=grasp_types,
                   surfaces=surfaces, sinks=sinks, stoves=stoves,
                   goal_conf=goal_conf, goal_on=goal_on, goal_cleaned=goal_cleaned, goal_cooked=goal_cooked,
                   body_names=body_names)

##################################################

JSON_DIRECTORY = 'problems/json/'

def get_json_directory():
    root_directory = os.path.dirname(os.path.abspath(__file__))
    return os.path.join(root_directory, '..', JSON_DIRECTORY)

def get_json_filenames():
    return os.listdir(get_json_directory())

def load_json_problem(problem_filename):
    reset_simulation()
    json_path = os.path.join(get_json_directory(), problem_filename)
    with open(json_path, 'r') as f:
        problem_json = json.loads(f.read())

    set_camera_pose(point_from_pose(parse_pose(problem_json['camera'])))

    task_json = problem_json['task']
    important_bodies = []
    for field in BODY_FIELDS:
        important_bodies.extend(task_json[field])

    robots = {robot['name']: parse_robot(robot) for robot in problem_json['robots']}
    bodies = {body['name']: parse_body(body, (body['name'] in important_bodies))
                      for body in problem_json['bodies']}
    regions = {region['name']: parse_region(region) for region in task_json['regions']}
    #print(get_image())

    return parse_task(task_json, robots, bodies, regions)

##################################################

# TODO: cannot solve any FFRob
FFROB = ['blocks_row', 'bury', 'clean', 'cook', 'dig',
         'invert', 'move', 'move_distractions',
         'organize', 'panel', 'real_regrasp',
         'regrasp', 'regrasp_distractions',
         'table', 'transporter', 'walls']

# ['blocked.json',
#  'dantam.json', 'dantam2.json', 'dantam_3.json', 'decision_namo.json',
#  'decision_wall_namo_2.json', 'decision_wall_namo_3.json',
#  'dinner.json', 'disconnected.json', 'distract_0.json', 'distract_10.json',
#  'distract_12.json', 'distract_16.json', 'distract_20.json', 'distract_24.json',
#  'distract_30.json', 'distract_4.json', 'distract_40.json', 'distract_8.json',
#  'distractions.json', 'easy_dinner.json',  'kitchen.json',
# 'move_regrasp.json', 'move_several_10.json',
#  'move_several_12.json', 'move_several_14.json', 'move_several_16.json', 'move_several_18.json',
#  'move_several_20.json', 'move_several_24.json', 'move_several_28.json', 'move_several_32.json',
#  'move_several_4.json', 'move_several_8.json', 'nonmonotonic_4_1.json', 'nonmonotonic_4_2.json',
#  'nonmonotonic_4_3.json', 'nonmonotonic_4_4.json', 'push_table.json',
#  'push_wall.json', 'rearrangement_10.json', 'rearrangement_12.json',
#  'rearrangement_14.json', 'rearrangement_16.json', 'rearrangement_18.json', 'rearrangement_20.json',
#  'rearrangement_4.json', 'rearrangement_6.json', 'rearrangement_8.json',
#  'replace.json', 'separate_2.json', 'separate_4.json', 'separate_6.json',
#  'separate_7.json', 'shelf_arrangement.json', 'simple.json', 'simple2.json', 'simple_holding.json',
#  'simple_namo.json', 'simple_push.json',
#  'srivastava_table.json', 'stacking.json',
#  'trivial_namo.json', 'two_tables_1.json', 'two_tables_2.json',
#  'two_tables_3.json', 'two_tables_4.json', 'wall_namo_2.json', 'wall_namo_3.json']

IJRR = []

# HBF
RSS = ['blocked.json',
 'dinner.json', 'disconnected.json', 'distract_0.json', 'distract_10.json',
 'distract_12.json', 'distract_16.json', 'distract_20.json', 'distract_24.json',
 'distract_30.json', 'distract_4.json', 'distract_40.json', 'distract_8.json',
 'distractions.json', 'easy_dinner.json',  'kitchen.json',
 'move_regrasp.json', 'move_several_10.json',
 'move_several_12.json', 'move_several_14.json', 'move_several_16.json', 'move_several_18.json',
 'move_several_20.json', 'move_several_24.json', 'move_several_28.json', 'move_several_32.json',
 'move_several_4.json', 'move_several_8.json', 'nonmonotonic_4_1.json', 'nonmonotonic_4_2.json', 'push_table.json',
 'push_wall.json', 'rearrangement_10.json', 'rearrangement_12.json',
 'rearrangement_14.json', 'rearrangement_16.json', 'rearrangement_18.json', 'rearrangement_20.json',
 'rearrangement_4.json', 'rearrangement_6.json', 'rearrangement_8.json',
 'replace.json', 'separate_2.json', 'separate_4.json', 'separate_6.json',
 'separate_7.json', 'shelf_arrangement.json', 'simple.json', 'simple2.json', 'simple_holding.json',
 'simple_push.json', 'srivastava_table.json', 'stacking.json',
 'two_tables_1.json', 'two_tables_2.json', 'two_tables_3.json', 'two_tables_4.json']

IJCAI = ['exists_hold_obs_0.json', 'exists_hold_obs_1.json',
 'exists_hold_obs_2.json', 'exists_hold_obs_4.json', 'exists_hold_obs_5.json', 'exists_holding_1.json',
 'exists_holding_2.json', 'exists_holding_3.json', 'exists_holding_blocked.json',
 'exists_holding_unreachable.json', 'exists_region_1.json',
 'sink_stove_2_16.json', 'sink_stove_2_20.json', 'sink_stove_2_4.json', 'sink_stove_2_8.json',
 'sink_stove_4_0.json', 'sink_stove_4_12.json', 'sink_stove_4_15.json', 'sink_stove_4_16.json',
 'sink_stove_4_20.json', 'sink_stove_4_30.json', 'sink_stove_4_4.json', 'sink_stove_4_40.json',
 'sink_stove_4_8.json']

##################################################

SCREENSHOT_DIR = 'images/json'

def main(screenshot=False):
    connect(use_gui=True)
    print(get_json_filenames())

    #problem_filenames = sorted(os.listdir(openrave_directory))
    #problem_filenames = ['{}.json'.format(name) for name in FFROB]
    #problem_filenames = ['sink_stove_4_30.json'] # 'dinner.json' | 'simple.json'
    problem_filenames = ['simple.json'] # 'dinner.json' | 'simple.json'

    # Mac width/height
    #width = 2560
    #height = 1600
    #
    #640, 480

    for problem_filename in problem_filenames:
        load_json_problem(problem_filename)
        if screenshot:
            problem_name = problem_filename.split('.')[0]
            image_filename = "{}.png".format(problem_name)
            image_path = os.path.join(SCREENSHOT_DIR, image_filename)
            wait_for_interrupt(max_time=0.5)
            os.system("screencapture -R {},{},{},{} {}".format(
                225, 200, 600, 500, image_path))
        else:
            wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()