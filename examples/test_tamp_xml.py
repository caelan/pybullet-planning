#!/usr/bin/env python

from __future__ import print_function

import colorsys
import glob
import os
import pybullet as p
import random
import sys

import numpy as np
from lxml import etree

from pybullet_tools.pr2_utils import DRAKE_PR2_URDF, set_group_conf
from pybullet_tools.utils import STATIC_MASS, CLIENT, connect, \
    disconnect, set_pose, wait_if_gui, load_model, HideOutput, base_values_from_pose, create_shape, \
    get_mesh_geometry, point_from_pose, set_camera_pose, draw_global_system
from pybullet_tools.utils import quaternion_from_matrix

# https://docs.python.org/3.5/library/xml.etree.elementtree.html
# https://lxml.de/tutorial.html

def parse_array(element):
    return np.array(element.text.split(), dtype=np.float)

def parse_pose(element):
    homogenous = [0, 0, 0, 1]
    matrix = np.reshape(np.concatenate([parse_array(element), homogenous]), [4, 4])
    point = matrix[:3, 3]
    quat = quaternion_from_matrix(matrix)
    return (point, quat)

def parse_boolean(element):
    text = element.text
    if text == 'true':
        return True
    if text == 'false':
        return False
    raise ValueError(text)

MAX_INT = sys.maxsize + 1

def parse_object(obj, mesh_directory):
    name = obj.find('name').text
    mesh_filename = obj.find('geom').text
    geom = get_mesh_geometry(os.path.join(mesh_directory, mesh_filename))
    pose = parse_pose(obj.find('pose'))
    movable = parse_boolean(obj.find('moveable'))

    color = (.75, .75, .75, 1)
    if 'red' in name:
        color = (1, 0, 0, 1)
    elif 'green' in name:
        color = (0, 1, 0, 1)
    elif 'blue' in name:
        color = (0, 0, 1, 1)
    elif movable: # TODO: assign new color
        #size = 2 * MAX_INT
        #size = 255
        #n = id(obj) % size
        #n = hash(obj) % size
        #h = float(n) / size
        h = random.random()
        r, g, b = colorsys.hsv_to_rgb(h, .75, .75)
        color = (r, g, b, 1)
    print(name, mesh_filename, movable, color)

    collision_id, visual_id = create_shape(geom, color=color)
    body_id = p.createMultiBody(baseMass=STATIC_MASS, baseCollisionShapeIndex=collision_id,
                                baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)
    set_pose(body_id, pose)

    return body_id

def parse_robot(robot):
    name = robot.find('name').text
    urdf = robot.find('urdf').text
    fixed_base = not parse_boolean(robot.find('movebase'))
    print(name, urdf, fixed_base)
    pose = parse_pose(robot.find('basepose'))
    torso = parse_array(robot.find('torso'))
    left_arm = parse_array(robot.find('left_arm'))
    right_arm = parse_array(robot.find('right_arm'))
    assert (name == 'pr2')

    with HideOutput():
        robot_id = load_model(DRAKE_PR2_URDF, fixed_base=True)
    set_pose(robot_id, pose)
    #set_group_conf(robot_id, 'base', base_values_from_pose(pose))
    set_group_conf(robot_id, 'torso', torso)
    set_group_conf(robot_id, 'left_arm', left_arm)
    set_group_conf(robot_id, 'right_arm', right_arm)
    #set_point(robot_id, Point(z=point_from_pose(pose)[2]))
    # TODO: base pose isn't right
    # print(robot.tag)
    # print(robot.attrib)
    # print(list(robot.iter('basepose')))
    return robot_id

DISC = 'disc'
BLOCK = 'cube'
PEG = 'peg'

def main():
    benchmark = 'tmp-benchmark-data'
    problem = 'problem1' # Hanoi
    #problem = 'problem2' # Blocksworld
    #problem = 'problem3' # Clutter
    #problem = 'problem4' # Nonmono

    root_directory = os.path.dirname(os.path.abspath(__file__))
    directory = os.path.join(root_directory, '..', 'problems', benchmark, problem)
    [mesh_directory] = list(filter(os.path.isdir, (os.path.join(directory, o)
                                                 for o in os.listdir(directory) if o.endswith('meshes'))))
    [xml_path] = [os.path.join(directory, o) for o in os.listdir(directory) if o.endswith('xml')]
    if os.path.isdir(xml_path):
        xml_path = glob.glob(os.path.join(xml_path, '*.xml'))[0]

    print(mesh_directory)
    print(xml_path)
    xml_data = etree.parse(xml_path)

    connect(use_gui=True)
    #add_data_path()
    #load_pybullet("plane.urdf")
    draw_global_system()
    set_camera_pose(camera_point=[+1, -1, 1])

    #root = xml_data.getroot()
    #print(root.items())
    for obj in xml_data.findall('/objects/obj'):
        parse_object(obj, mesh_directory)
    for robot in xml_data.findall('/robots/robot'):
        parse_robot(robot)
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    main()