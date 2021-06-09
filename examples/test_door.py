#!/usr/bin/env python

from __future__ import print_function

import os
import pybullet as p
import time

from pybullet_tools.utils import add_data_path, connect, disconnect, wait_if_gui, load_pybullet, \
    HideOutput, draw_global_system, dump_body, get_sample_fn, get_movable_joints, enable_gravity, step_simulation, \
    get_time_step, elapsed_time, get_joint_intervals, \
    velocity_control_joints, PI, set_point, Point, get_box_geometry, \
    create_shape, set_camera_pose, Pose, get_all_links, base_aligned_z, BLACK, LinkInfo, create_multi_body


# bullet3/examples/pybullet/examples
# experimentalCcdSphereRadius.py
# heightfield.py
# signedDistanceField.py


def create_door(width=0.1, length=1, height=2, mass=1, handle=True, **kwargs):
    # TODO: frame, hinge, cylinder on end, sliding door, handle, knob
    geometry = get_box_geometry(width, length, height)
    door_collision, door_visual = create_shape(
        geometry, pose=Pose(Point(y=-length/2., z=height/2.)), **kwargs)
    door_link = LinkInfo(mass=mass, collision_id=door_collision, visual_id=door_visual,
                         parent=0, joint_type=p.JOINT_REVOLUTE, joint_axis=[0, 0, 1])

    links = [door_link]
    if handle:
        side = 0.05
        geometry = get_box_geometry(width=side, length=side, height=5*side)

        #collision_id, visual_id = create_shape_array(geoms, poses, colors)

        handle_collision, handle_visual = create_shape(
            geometry, pose=Pose(Point(x=width/2+side/2, y=-3*length / 4., z=height / 2.)), color=BLACK)
        handle_link = LinkInfo(mass=mass, collision_id=handle_collision, visual_id=handle_visual,
                               parent=1, joint_type=p.JOINT_FIXED)
        links.append(handle_link)

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