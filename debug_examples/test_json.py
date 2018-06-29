#!/usr/bin/env python

from __future__ import print_function

import json
import os

from pybullet_tools.utils import create_box_shape, STATIC_MASS, CLIENT, user_input, connect, \
    disconnect, set_point, set_quat, set_pose, wait_for_interrupt, load_model, set_joint_position, \
    joint_from_name, has_joint, create_cylinder_shape, get_bodies, HideOutput, base_values_from_pose, get_image
from pybullet_tools.pr2_utils import DRAKE_PR2_URDF, set_group_conf
import pybullet as p
import numpy as np

def parse_point(point_json):
    return tuple(point_json[key] for key in ['x', 'y', 'z'])

def parse_quat(quat_json):
    return tuple(quat_json[key] for key in ['x', 'y', 'z', 'w'])

def parse_pose(pose_json):
    return parse_point(pose_json['point']), parse_quat(pose_json['quat'])

def load_robot(robot_json):
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

def main():
    root_directory = os.path.dirname(os.path.abspath(__file__))
    openrave_directory = os.path.join(root_directory, '..', 'problems')
    rel_path = 'simple.json'
    path = os.path.join(openrave_directory, rel_path)

    with open(path, 'r') as f:
        data = json.loads(f.read())

    connect(use_gui=True)
    body_from_name = {}
    for body in data['bodies']:
        for link in body['links']:
            print(link)
            #for geometry in link['geometries']:
            for geometry in link:
                # TODO: can also just make fixed lengths

                r, g, b, a = geometry['color']
                a = 1 - a
                color = [r, g, b, a]
                if geometry['type'] == 'box':
                    w, l, h = 2*np.array(geometry['extents'])
                    # createCollisionShapeArray
                    collision_id, visual_id = create_box_shape(w, l, h, pose=parse_pose(geometry), color=color)
                elif geometry['type'] == 'cylinder':
                    collision_id, visual_id = create_cylinder_shape(geometry['radius'], geometry['height'],
                                                                    pose=parse_pose(geometry), color=color)
                else:
                    continue
                body_id = p.createMultiBody(baseMass=STATIC_MASS, baseCollisionShapeIndex=collision_id,
                                            baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)
                set_pose(body_id, parse_pose(body))
                #set_point(body_id, parse_point(body['point']))
                #set_quat(body_id, parse_quat(body['quat']))
                body_from_name[body['name']] = body_id
    print(get_bodies())


    for robot in data['robots']:
        load_robot(robot)

    #print(get_image())




    wait_for_interrupt()
    disconnect()


if __name__ == '__main__':
    main()