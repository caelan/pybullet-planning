#!/usr/bin/env python

from __future__ import print_function

import json
import os

from pybullet_tools.utils import STATIC_MASS, CLIENT, user_input, connect, \
    disconnect, set_point, set_quat, set_pose, wait_for_interrupt, load_model, set_joint_position, \
    joint_from_name, has_joint, get_bodies, HideOutput, base_values_from_pose, set_camera, \
    get_image, create_shape, create_shape_array, get_box_geometry, get_cylinder_geometry, \
    Point, Pose, NULL_ID, euler_from_quat, quat_from_pose, point_from_pose, set_camera_pose
from pybullet_tools.pr2_utils import DRAKE_PR2_URDF, set_group_conf
import pybullet as p
import numpy as np

def parse_point(point_json):
    return tuple(point_json[key] for key in ['x', 'y', 'z'])

def parse_quat(quat_json):
    return tuple(quat_json[key] for key in ['x', 'y', 'z', 'w'])

def parse_pose(pose_json):
    return parse_point(pose_json['point']), parse_quat(pose_json['quat'])

def parse_color(color_json):
    return color_json
    #return parse_point(pose_json['point']), parse_quat(pose_json['quat'])

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
    return robot_id

def parse_region(region):
    lower = np.min(region['hull'], axis=0)
    upper = np.max(region['hull'], axis=0)
    x, y = (lower + upper) / 2.
    w, h = (upper - lower)  # / 2.
    geom = get_box_geometry(w, h, 1e-3)
    _, visual_id = create_shape(geom, pose=Pose(Point(x, y)), color=parse_color(region['rgba']))
    region_id = p.createMultiBody(baseMass=STATIC_MASS, baseCollisionShapeIndex=NULL_ID,
                                  baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)
    set_pose(region_id, parse_pose(region))
    return region_id

def parse_body(body):
    for link in body['links']:
        # for geometry in link['geometries']:
        geoms = []
        poses = []
        colors = []
        for geometry in link:
            # TODO: can also just make fixed lengths
            r, g, b, a = geometry['rgba']
            a = 1 - a
            color = [r, g, b, a]
            if geometry['type'] == 'box':
                geoms.append(get_box_geometry(*2 * np.array(geometry['extents'])))
            elif geometry['type'] == 'cylinder':
                geoms.append(get_cylinder_geometry(geometry['radius'], geometry['height']))
            else:
                raise NotImplementedError(geometry['type'])
            poses.append(parse_pose(geometry))
            colors.append(parse_color(color))

        collision_id, visual_id = create_shape_array(geoms, poses, colors)
        print(body['name'], colors)
        # collision_id, visual_id = create_shape(geom, pose=parse_pose(geometry),
        #                                       color=color) #, specular=geometry['specular'])
        body_id = p.createMultiBody(baseMass=STATIC_MASS, baseCollisionShapeIndex=collision_id,
                                    baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)
        set_pose(body_id, parse_pose(body))

def main():
    root_directory = os.path.dirname(os.path.abspath(__file__))
    openrave_directory = os.path.join(root_directory, '..', 'problems')
    rel_path = 'simple.json'
    path = os.path.join(openrave_directory, rel_path)

    with open(path, 'r') as f:
        data = json.loads(f.read())

    connect(use_gui=True)

    camera_pose = parse_pose(data['camera'])
    set_camera_pose(point_from_pose(camera_pose))

    region_from_name = {}
    for region in data['task']['regions']:
        region_from_name[region['name']] = parse_region(region)

    body_from_name = {}
    for body in data['bodies']:
        body_from_name[body['name']] = parse_body(body)

    robot = parse_robot(data['robots'][0])
    #print(get_image())

    wait_for_interrupt()
    disconnect()


if __name__ == '__main__':
    main()