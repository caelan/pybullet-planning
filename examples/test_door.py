#!/usr/bin/env python

from __future__ import print_function

from collections import namedtuple

import os
import pybullet as p
import time

from pybullet_tools.utils import add_data_path, connect, disconnect, wait_if_gui, set_camera, load_pybullet, \
    HideOutput, draw_global_system, dump_body, get_sample_fn, get_movable_joints, set_joint_positions, \
    enable_gravity, step_simulation, GRAVITY, get_time_step, elapsed_time, control_joints, get_joint_intervals, \
    velocity_control_joints, PI, set_point, Point, STATIC_MASS, NULL_ID, unit_point, unit_quat, get_box_geometry, \
    create_shape, RED, BASE_LINK, set_camera_pose, Pose, get_aabb, approximate_as_prism, get_all_links, get_aabb_center, \
    get_point, get_difference, set_joint_limits, set_collision_margin, base_aligned_z

# bullet3/examples/pybullet/examples
# experimentalCcdSphereRadius.py
# heightfield.py
# signedDistanceField.py

LINK_INFO_DEFAULT = [('mass', STATIC_MASS), ('collision_id', NULL_ID), ('visual_id', NULL_ID),
                     ('point', unit_point()), ('quat', unit_quat()),
                     ('inertial_point', unit_point()), ('inertial_quat', unit_quat()),
                     ('parent', 0), ('joint_type', p.JOINT_FIXED), ('joint_axis', unit_point())]

def named_tuple(name, fields, defaults=None):
    NT = namedtuple(name, fields)
    if defaults is not None:
        assert len(fields) == len(defaults)
        NT.__new__.__defaults__ = defaults
    return NT

LinkInfo = named_tuple('LinkInfo', *zip(*LINK_INFO_DEFAULT))

def dict_from_kwargs(**kwargs):
    return kwargs


def create_multi_body(base_link=None, links=[]):
    assert base_link or links
    if base_link is None:
        base_link = LinkInfo()
    masses = [link.mass for link in links]
    collision_ids = [link.collision_id for link in links]
    visual_ids = [link.visual_id for link in links]
    points = [link.point for link in links]
    quats = [link.quat for link in links]
    inertial_points = [link.inertial_point for link in links]
    inertial_quats = [link.inertial_quat for link in links]
    parents = [link.parent for link in links]
    joint_types = [link.joint_type for link in links]
    joint_axes = [link.joint_axis for link in links]
    return p.createMultiBody(
        baseMass=base_link.mass,
        baseCollisionShapeIndex=base_link.collision_id,
        baseVisualShapeIndex=base_link.visual_id,
        basePosition=base_link.point,
        baseOrientation=base_link.quat,
        # baseInertialFramePosition=base_link.inertial_point,
        # baseInertialFrameOrientation=base_link.inertial_quat,
        linkMasses=masses,
        linkCollisionShapeIndices=collision_ids,
        linkVisualShapeIndices=visual_ids,
        linkPositions=points,
        linkOrientations=quats,
        linkInertialFramePositions=inertial_points,
        linkInertialFrameOrientations=inertial_quats,
        linkParentIndices=parents,
        linkJointTypes=joint_types,
        linkJointAxis=joint_axes,
        #physicsClientId=CLIENT,
    )


def create_door(width=0.1, length=1, height=2, mass=1, **kwargs):
    # TODO: frame, hinge, cylinder on end, sliding door, handle, knob
    geometry = get_box_geometry(width, length, height)
    door_collision, door_visual = create_shape(
        geometry, pose=Pose(Point(y=-length/2., z=height/2.)), color=RED)# , **kwargs)
    door_link = LinkInfo(mass=mass, collision_id=door_collision, visual_id=door_visual,
                         parent=0, joint_type=p.JOINT_REVOLUTE, joint_axis=[0, 0, 1])
    links = [door_link]
    body = create_multi_body(links=links)
    #set_joint_limits(body, link=0, lower=-PI, upper=PI)
    return body

def main():
    connect(use_gui=True)
    data_path = add_data_path()
    print(data_path)
    print(sorted(name for name in os.listdir(data_path) if os.path.isdir(os.path.join(data_path, name))))
    print(sorted(name for name in os.listdir(data_path) if not os.path.isdir(
        os.path.join(data_path, name)) and (os.path.splitext(name)[1] not in ['.png', '.gif', '.jpeg', '.py', '.mtl'])))
    # TODO: recursively search

    robot_path = os.path.abspath(os.path.join(data_path, os.pardir, 'pybullet_robots'))
    print(robot_path)

    draw_global_system()
    set_camera_pose(camera_point=Point(-2, -2, +2))

    with HideOutput():
        plane = load_pybullet('plane.urdf', fixed_base=True)
        #plane = load_model('plane.urdf')
        set_point(plane, Point(z=-1e-3))

        #door = load_pybullet('models/door.urdf', fixed_base=True) # From drake
        #set_point(door, Point(z=-.1))
        door = create_door()

        door_joints = get_movable_joints(door)
        door_links = get_all_links(door)
        set_point(door, Point(z=base_aligned_z(door)))
        #set_collision_margin(door, link=0, margin=0.)

    dump_body(door)
    sample_fn = get_sample_fn(door, door_joints)
    #set_joint_positions(door, door_joints, sample_fn())
    # while True:
    #     positions = sample_fn()
    #     set_joint_positions(door, door_joints, positions)
    #     wait_if_gui()

    lower, upper = get_joint_intervals(door, door_joints)
    #control_joints(door, door_joints, positions=lower)
    velocity_control_joints(door, door_joints, velocities=[PI / 4]) # Able to exceed limits

    wait_if_gui('Begin?')
    enable_gravity()
    #p.setGravity(-GRAVITY, 0, 0)
    dt = get_time_step()
    print(dt)
    steps = 0
    start_time = time.time()
    while True:
        step_simulation()
        steps += 1
        print('Step: {} | Sim time: {:.3f} | Elapsed time: {:.3f}'.format(
            steps, steps*dt, elapsed_time(start_time)))
        time.sleep(dt/2.)


    wait_if_gui('Finish?')
    disconnect()

if __name__ == '__main__':
    main()