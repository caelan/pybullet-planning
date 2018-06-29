#!/usr/bin/env python

from __future__ import print_function

import json
import os

from pybullet_tools.utils import STATIC_MASS, CLIENT, user_input, connect, \
    disconnect, set_point, set_quat, set_pose, wait_for_interrupt, load_model, set_joint_position, \
    joint_from_name, has_joint, get_bodies, HideOutput, base_values_from_pose, set_camera, \
    get_image, create_shape, create_shape_array, get_box_geometry, get_cylinder_geometry, \
    Point, Pose, NULL_ID, euler_from_quat, quat_from_pose, point_from_pose, set_camera_pose, \
    get_sphere_geometry, reset_simulation
from pybullet_tools.pr2_utils import DRAKE_PR2_URDF, set_group_conf, REST_LEFT_ARM, rightarm_from_leftarm
import pybullet as p
import numpy as np

def parse_point(point_json):
    return tuple(point_json[key] for key in ['x', 'y', 'z'])

def parse_quat(quat_json):
    return tuple(quat_json[key] for key in ['x', 'y', 'z', 'w'])

def parse_pose(pose_json):
    return parse_point(pose_json['point']), parse_quat(pose_json['quat'])

def parse_color(color_json):
    return tuple(color_json[key] for key in ['r', 'g', 'b', 'a'])

def parse_robot(robot_json):
    pose = parse_pose(robot_json)
    if robot_json['name'] == 'pr2':
        with HideOutput(True):
            robot_id = load_model(DRAKE_PR2_URDF, fixed_base=True)
        set_group_conf(robot_id, 'base',  base_values_from_pose(pose))
    else:
        #set_pose(robot_id, pose)
        raise NotImplementedError(robot_json['name'])

    for joint, values in robot_json['conf'].items():
        [value] = values
        if has_joint(robot_id, joint):
            set_joint_position(robot_id, joint_from_name(robot_id, joint), value)
        else:
            print('Robot {} lacks joint {}'.format(robot_json['name'], joint))

    if robot_json['name'] == 'pr2':
        set_group_conf(robot_id, 'torso', [0.2])
        set_group_conf(robot_id, 'left_arm', REST_LEFT_ARM)
        set_group_conf(robot_id, 'right_arm', rightarm_from_leftarm(REST_LEFT_ARM))

    return robot_id

def parse_region(region):
    lower = np.min(region['hull'], axis=0)
    upper = np.max(region['hull'], axis=0)
    x, y = (lower + upper) / 2.
    w, h = (upper - lower)  # / 2.
    geom = get_box_geometry(w, h, 1e-3)
    _, visual_id = create_shape(geom, pose=Pose(Point(x, y)), color=parse_color(region['color']))
    region_id = p.createMultiBody(baseMass=STATIC_MASS, baseCollisionShapeIndex=NULL_ID,
                                  baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)
    set_pose(region_id, parse_pose(region))
    return region_id

def parse_body(body, important=False):
    [link] = body['links']
    # for geometry in link['geometries']:
    geoms = []
    poses = []
    colors = []
    skipped = False
    for geometry in link:
        # TODO: can also just make fixed links
        if geometry['type'] == 'box':
            geoms.append(get_box_geometry(*2 * np.array(geometry['extents'])))
        elif geometry['type'] == 'cylinder':
            geoms.append(get_cylinder_geometry(geometry['radius'], geometry['height']))
        elif geometry['type'] == 'sphere':
            # TODO: does sphere not work?
            geoms.append(get_sphere_geometry(geometry['radius']))
        elif geometry['type'] == 'trimesh':
            skipped = True
            continue
        else:
            raise NotImplementedError(geometry['type'])
        poses.append(parse_pose(geometry))
        colors.append(parse_color(geometry['color'])) # specular=geometry['specular'])

    if skipped:
        if important:
            center = body['aabb']['center']
            extents = 2*np.array(body['aabb']['extents'])
            geoms = [get_box_geometry(*extents)]
            poses = [Pose(center)]
            colors = [(.5, .5, .5, 1)]
        else:
            return None
    if not geoms:
        return None
    if len(geoms) == 1:
        collision_id, visual_id = create_shape(geoms[0], pose=poses[0], color=colors[0])
    else:
        collision_id, visual_id = create_shape_array(geoms, poses, colors)
    body_id = p.createMultiBody(baseMass=STATIC_MASS, baseCollisionShapeIndex=collision_id,
                                baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)
    set_pose(body_id, parse_pose(body))
    return body_id

BODY_FIELDS = [u'table_names', u'sink_names', u'stove_names']

# [u'goal_cleaned', u'regions', u'goal_stackings', u'name', u'sink_names', u'goal_regions', u'goal_cooked', u'goal_config', u'table_names', u'floor_object_names', u'grasp_types', u'goal_holding', u'object_names', u'stove_names', u'goal_poses']

def load(path):
    reset_simulation()
    with open(path, 'r') as f:
        problem = json.loads(f.read())

    camera_pose = parse_pose(problem['camera'])
    set_camera_pose(point_from_pose(camera_pose))

    task = problem['task']
    important_bodies = []
    for field in BODY_FIELDS:
        important_bodies.extend(task[field])

    region_from_name = {}
    for region in task['regions']:
        region_from_name[region['name']] = parse_region(region)

    body_from_name = {}
    for body in problem['bodies']:
        name = body['name']
        body_from_name[name] = parse_body(body, (name in important_bodies))

    robot = parse_robot(problem['robots'][0])
    #print(get_image())


def main():
    root_directory = os.path.dirname(os.path.abspath(__file__))
    openrave_directory = os.path.join(root_directory, '../problems/json')
    connect(use_gui=True)

    problem_filenames = sorted(os.listdir(openrave_directory))
    #problem_filenames = ['sink_stove_4_30.json'] # 'dinner.json' | 'simple.json'

    for problem_filename in problem_filenames:
        path = os.path.join(openrave_directory, problem_filename)
        print(path)
        load(path)
        wait_for_interrupt()
    disconnect()

if __name__ == '__main__':
    main()