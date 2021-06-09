#!/usr/bin/env python

from __future__ import print_function

import os
import pybullet as p
import time
import argparse

from pybullet_tools.utils import add_data_path, connect, disconnect, wait_if_gui, load_pybullet, \
    HideOutput, draw_global_system, dump_body, get_sample_fn, get_movable_joints, enable_gravity, step_simulation, \
    get_time_step, elapsed_time, get_joint_intervals, \
    velocity_control_joints, PI, set_point, Point, get_box_geometry, \
    create_shape, set_camera_pose, Pose, get_all_links, base_aligned_z, BLACK, LinkInfo, create_multi_body, \
    Shape, create_shape_array, unzip, STATIC_MASS, get_aabb_extent, get_aabb, set_position, base_aligned, \
    create_box, BLUE, set_velocity, add_pose_constraint, get_pose, synchronize_viewer, get_velocity, get_bodies, \
    get_distance, get_point, set_renderer


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
    parser = argparse.ArgumentParser()
    parser.add_argument('-video', action='store_true')
    args = parser.parse_args()
    video = 'video.mp4' if args.video else None

    connect(use_gui=True, mp4=video)
    set_renderer(enable=False)
    data_path = add_data_path()
    print(data_path)
    print(sorted(name for name in os.listdir(data_path) if os.path.isdir(os.path.join(data_path, name))))
    print(sorted(name for name in os.listdir(data_path) if not os.path.isdir(
        os.path.join(data_path, name)) and (os.path.splitext(name)[1] not in ['.png', '.gif', '.jpeg', '.py', '.mtl'])))
    # TODO: recursively search

    robot_path = os.path.abspath(os.path.join(data_path, os.pardir, 'pybullet_robots'))
    print(robot_path)

    draw_global_system()
    set_camera_pose(camera_point=Point(+2, -2, +2))

    with HideOutput():
        plane = load_pybullet('plane.urdf', fixed_base=True)
        #plane = load_model('plane.urdf')
        set_point(plane, Point(z=-1e-3))

        #door = load_pybullet('models/door.urdf', fixed_base=True) # From drake
        #set_point(door, Point(z=-.1))
        door = create_door()

        door_joints = get_movable_joints(door)
        door_links = get_all_links(door)
        #set_position(door, z=base_aligned_z(door))
        set_point(door, base_aligned(door))
        #set_collision_margin(door, link=0, margin=0.)

        side = 0.25
        robot = create_box(w=side, l=side, h=side, mass=5., color=BLUE)
        set_point(robot, Point(x=+2, z=base_aligned_z(robot)))
        #set_velocity(robot, linear=Point(x=-1))

    bodies = get_bodies()

    dump_body(door)
    sample_fn = get_sample_fn(door, door_joints)
    #set_joint_positions(door, door_joints, sample_fn())
    # while True:
    #     positions = sample_fn()
    #     set_joint_positions(door, door_joints, positions)
    #     wait_if_gui()

    #lower, upper = get_joint_intervals(door, door_joints)
    #control_joints(door, door_joints, positions=lower)
    #velocity_control_joints(door, door_joints, velocities=[PI / 4]) # Able to exceed limits

    target_point, target_quat = map(list, get_pose(robot))
    target_point[0] = -2
    add_pose_constraint(robot, pose=(target_point, target_quat), max_force=200) # TODO: velocity constraint?

    if video is None:
        wait_if_gui('Begin?')
    set_renderer(enable=True)
    enable_gravity()
    #p.setGravity(-GRAVITY, 0, 0)
    dt = get_time_step()
    print('Time step:', dt)
    #sleep_per_step = dt/2.
    sleep_per_step = 0.01

    steps = 0
    start_time = time.time()
    while True:
        step_simulation()
        synchronize_viewer()
        steps += 1
        print('Step: {} | Sim time: {:.3f} | Elapsed time: {:.3f}'.format(
            steps, steps*dt, elapsed_time(start_time)))
        #if video is None:
        #   time.sleep(sleep_per_step)
        if get_distance(target_point, get_point(robot)) < 1e-3: # TODO: velocity condition
            break

    if video is None:
        wait_if_gui('Finish?')
    disconnect()

if __name__ == '__main__':
    main()
