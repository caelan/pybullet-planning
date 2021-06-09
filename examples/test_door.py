#!/usr/bin/env python

from __future__ import print_function

import os
import pybullet as p
import time

from pybullet_tools.utils import add_data_path, connect, disconnect, wait_if_gui, load_pybullet, \
    HideOutput, draw_global_system, dump_body, get_sample_fn, get_movable_joints, enable_gravity, step_simulation, \
    get_time_step, elapsed_time, get_joint_intervals, \
    velocity_control_joints, PI, set_point, Point, get_box_geometry, \
    create_shape, set_camera_pose, Pose, get_all_links, base_aligned_z, BLACK, LinkInfo, create_multi_body, \
    Shape, create_shape_array, unzip, STATIC_MASS


# bullet3/examples/pybullet/examples
# experimentalCcdSphereRadius.py
# heightfield.py
# signedDistanceField.py


def create_door(width=0.08, length=1, height=2, mass=1, handle=True, frame=True, **kwargs):
    # TODO: hinge, cylinder on end, sliding door, knob
    geometry = get_box_geometry(width, length, height)
    hinge = 0 # -width/2
    door_collision, door_visual = create_shape(
        geometry, pose=Pose(Point(x=hinge, y=-length/2., z=height/2.)), **kwargs)
    door_link = LinkInfo(mass=mass, collision_id=door_collision, visual_id=door_visual,
                         parent=0, joint_type=p.JOINT_REVOLUTE, joint_axis=[0, 0, 1])

    links = [door_link]
    if handle:
        side = 0.04
        handle_height = 5*side
        point = Point(x=width/2+side/2+side, y=-3*length / 4., z=height / 2.)
        shapes = [
            Shape(get_box_geometry(width=side, length=side, height=handle_height), color=BLACK),
            Shape(get_box_geometry(width=side, length=side, height=side),
                  pose=Pose(Point(x=-side, z=(handle_height - side)/2.)), color=BLACK),
            Shape(get_box_geometry(width=side, length=side, height=side),
                  pose=Pose(Point(x=-side, z=-(handle_height - side)/2.)), color=BLACK),
        ]
        handle_collision, handle_visual = create_shape_array(*unzip(shapes))
        handle_link = LinkInfo(mass=1, collision_id=handle_collision, visual_id=handle_visual,
                               point=point, parent=1, joint_type=p.JOINT_FIXED)
        links.append(handle_link)

    if frame:
        # TODO: could be part of the base link instead
        pad = 0.1
        #point = Point(x=width/2+side/2+side, y=-3*length / 4., z=height / 2.)
        shapes = [
            Shape(get_box_geometry(width=width, length=length+2*pad, height=pad),
                  pose=Pose(Point(z=height/2.+pad/2.)), color=BLACK),
            Shape(get_box_geometry(width=width, length=pad, height=height),
                  pose=Pose(Point(y=(length + pad)/2.)), color=BLACK),
            Shape(get_box_geometry(width=width, length=pad, height=height),
                  pose=Pose(Point(y=-(length + pad) / 2.)), color=BLACK),
        ]
        frame_collision, frame_visual = create_shape_array(*unzip(shapes))
        frame_link = LinkInfo(mass=STATIC_MASS, collision_id=frame_collision, visual_id=frame_visual,
                              point=Point(y=-length/2., z=height/2.),
                              parent=0, joint_type=p.JOINT_FIXED)
        links.append(frame_link)

    body = create_multi_body(links=links)
    #draw_circle(center=unit_point(), radius=width/2., parent=body, parent_link=0)
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