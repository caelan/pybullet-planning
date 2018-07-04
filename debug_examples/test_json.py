#!/usr/bin/env python

from __future__ import print_function

import json
import os

from pybullet_tools.parse_json import parse_pose, parse_robot, parse_region, parse_body
from pybullet_tools.utils import connect, \
    disconnect, wait_for_interrupt, point_from_pose, set_camera_pose, \
    reset_simulation
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

    arms = [arm]
    movable = [bodies[n] for n in task['object_names']]
    grasp_types = task['grasp_types']
    surfaces = [bodies[n] for n in task['table_names']] + regions.values()
    sinks = [bodies[n] for n in task['sink_names']]
    stoves = [bodies[n] for n in task['stove_names']]
    #buttons = tuple()
    assert task['goal_config'] is None
    goal_conf = None
    goal_on = [(bodies[n1], bodies[n2]) for n1, n2 in task['goal_stackings'].items()] + \
              [(bodies[n1], regions[n2]) for n1, n2 in task['goal_regions'].items()]
    goal_cleaned = [bodies[n] for n in task['goal_cleaned']]
    goal_cooked = [bodies[n] for n in task['goal_cooked']]

    print(task)
    return Problem(robot=robot, arms=arms, movable=movable, grasp_types=grasp_types,
                   surfaces=surfaces, sinks=sinks, stoves=stoves,
                   goal_conf=goal_conf, goal_on=goal_on, goal_cleaned=goal_cleaned, goal_cooked=goal_cooked)

def load(problem_filename):
    root_directory = os.path.dirname(os.path.abspath(__file__))
    json_directory = os.path.join(root_directory, '../problems/json')
    path = os.path.join(json_directory, problem_filename)

    reset_simulation()
    with open(path, 'r') as f:
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

FFROB = ['blocks_row', 'bury', 'clean', 'cook', 'dig',
         'invert', 'move', 'move_distractions',
         'organize', 'panel', 'real_regrasp',
         'regrasp', 'regrasp_distractions',
         'table', 'transporter', 'walls']

SCREENSHOT_DIR = 'images/json'

def main():

    connect(use_gui=True)

    #problem_filenames = sorted(os.listdir(openrave_directory))
    #problem_filenames = ['{}.json'.format(name) for name in FFROB]
    #problem_filenames = ['sink_stove_4_30.json'] # 'dinner.json' | 'simple.json'
    problem_filenames = ['simple.json'] # 'dinner.json' | 'simple.json'

    # Mac width/height
    #width = 2560
    #height = 1600
    #
    #640, 480
    screenshot = False

    for problem_filename in problem_filenames:
        load(problem_filename)
        if screenshot:
            problem_name = problem_filename.split('.')[0]
            image_filename = "{}.png".format(problem_name)
            image_path = os.path.join(SCREENSHOT_DIR, image_filename)
            wait_for_interrupt(max_time=0.5)
            os.system("screencapture -R {},{},{},{} {}".format(
                225, 200, 600, 500, image_path))
        else:
            wait_for_interrupt()
    disconnect()

if __name__ == '__main__':
    main()