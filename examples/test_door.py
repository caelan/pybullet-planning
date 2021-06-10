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
    get_distance, get_point, set_renderer, get_cylinder_geometry, pairwise_collision, TURTLEBOT_URDF, set_joint_positions, \
    set_all_color, control_joint, irange, INF, control_joints, get_first_link, point_from_pose, get_link_pose, \
    get_joint_velocities, get_max_velocities, get_max_force, get_max_forces, get_joint_torques, read
#from examples.test_turtlebot_motion import BASE_JOINTS

# bullet3/examples/pybullet/examples
# experimentalCcdSphereRadius.py
# heightfield.py
# signedDistanceField.py


def create_door(width=0.08, length=1, height=2, mass=1, handle=True, frame=True, **kwargs):
    # TODO: hinge, cylinder on end, sliding door, knob
    # TODO: self collisions
    geometry = get_box_geometry(width, length, height)
    hinge = 0 # -width/2
    door_collision, door_visual = create_shape(
        geometry, pose=Pose(Point(x=hinge, y=-length/2., z=height/2.)), **kwargs)
    door_link = LinkInfo(mass=mass, collision_id=door_collision, visual_id=door_visual,
                         parent=0, joint_type=p.JOINT_REVOLUTE, joint_axis=[0, 0, 1])

    links = [door_link]
    if handle:
        diameter = 0.04
        handle_height = 5*diameter
        point = Point(x=width/2+diameter/2+diameter, y=-3*length / 4., z=height / 2.)
        shapes = [
            Shape(get_cylinder_geometry(radius=diameter/2, height=handle_height), color=BLACK),
            Shape(get_cylinder_geometry(radius=diameter/2, height=diameter),
                  pose=Pose(Point(x=-diameter, z=(handle_height - diameter)/2.)), color=BLACK),
            Shape(get_cylinder_geometry(radius=diameter/2, height=diameter),
                  pose=Pose(Point(x=-diameter, z=-(handle_height - diameter)/2.)), color=BLACK),
        ]
        handle_collision, handle_visual = create_shape_array(*unzip(shapes))
        handle_link = LinkInfo(mass=1, collision_id=handle_collision, visual_id=handle_visual,
                               point=point, parent=1, joint_type=p.JOINT_FIXED)
        links.append(handle_link)

    if frame:
        # TODO: could be part of the base link instead
        side = 0.1
        shapes = [
            Shape(get_box_geometry(width=width, length=length+2*side, height=side),
                  pose=Pose(Point(z=height/2.+side/2.)), color=BLACK),
            Shape(get_box_geometry(width=width, length=side, height=height),
                  pose=Pose(Point(y=(length + side)/2.)), color=BLACK),
            Shape(get_box_geometry(width=width, length=side, height=height),
                  pose=Pose(Point(y=-(length + side) / 2.)), color=BLACK),
        ]
        frame_collision, frame_visual = create_shape_array(*unzip(shapes))
        frame_link = LinkInfo(mass=STATIC_MASS, collision_id=frame_collision, visual_id=frame_visual,
                              point=Point(y=-length/2., z=height/2.),
                              parent=0, joint_type=p.JOINT_FIXED)
        links.append(frame_link)

    body = create_multi_body(links=links)
    #draw_circle(center=unit_point(), diameter=width/2., parent=body, parent_link=0)
    #set_joint_limits(body, link=0, lower=-PI, upper=PI)
    return body

IGNORE_EXT = ['.png', '.gif', '.jpeg', '.py', '.mtl']

def main(use_turtlebot=True):
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
        os.path.join(data_path, name)) and (os.path.splitext(name)[1] not in IGNORE_EXT)))
    # TODO: recursively search

    robot_path = os.path.abspath(os.path.join(data_path, os.pardir, 'pybullet_robots'))
    print(robot_path)

    draw_global_system()
    set_camera_pose(camera_point=Point(+1.5, -1.5, +1.5), target_point=Point(-1.5, +1.5, 0))

    start_x = +2
    target_x = -2

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

    if not use_turtlebot:
        side = 0.25
        robot = create_box(w=side, l=side, h=side, mass=5., color=BLUE)
        set_position(robot, x=start_x)
        #set_velocity(robot, linear=Point(x=-1))
    else:
        print(os.path.abspath(TURTLEBOT_URDF))
        #print(read(TURTLEBOT_URDF))
        robot = load_pybullet(TURTLEBOT_URDF, merge=True, fixed_base=True)
        robot_joints = get_movable_joints(robot)[:3]
        set_joint_positions(robot, robot_joints, [start_x, 0, PI])
    set_all_color(robot, BLUE)
    set_position(robot, z=base_aligned_z(robot))
    robot_link = get_first_link(robot)

    #all_bodies = get_bodies()
    dump_body(door)
    dump_body(robot)
    #sample_fn = get_sample_fn(door, door_joints)
    #set_joint_positions(door, door_joints, sample_fn())
    # while True:
    #     positions = sample_fn()
    #     set_joint_positions(door, door_joints, positions)
    #     wait_if_gui()

    #lower, upper = get_joint_intervals(door, door_joints)
    #control_joints(door, door_joints, positions=lower)
    #velocity_control_joints(door, door_joints, velocities=[PI / 4]) # Able to exceed limits

    if not use_turtlebot:
        target_point, target_quat = map(list, get_pose(robot))
        target_point[0] = target_x
        add_pose_constraint(robot, pose=(target_point, target_quat), max_force=200) # TODO: velocity constraint?
    else:
        # p.changeDynamics(robot, robot_joints[0], # Doesn't work
        #                  maxJointVelocity=1,
        #                  jointLimitForce=1,)
        print('Max velocities:', get_max_velocities(robot, robot_joints))
        print('Max forces:', get_max_forces(robot, robot_joints))
        control_joint(robot, robot_joints[0], position=target_x, velocity=0,
                      position_gain=None, velocity_scale=None, max_velocity=100, max_force=300)
        #control_joints(robot, robot_joints, positions=[target_x, 0, PI], max_force=300)
        #velocity_control_joints(robot, robot_joints, velocities=[-2., 0, 0]) #, max_force=300)

    set_renderer(enable=True)
    if video is None:
        wait_if_gui('Begin?')
    enable_gravity()
    #p.setGravity(-GRAVITY, 0, 0)
    dt = get_time_step()
    print('Time step: {:.6f} sec'.format(dt))
    #sleep_per_step = dt/2.
    #sleep_per_step = 0.01

    start_time = last_print = time.time()
    for step in irange(INF):
        step_simulation()
        synchronize_viewer()
        if elapsed_time(last_print) >= 1:
            last_print = time.time()
            print('Step: {} | Sim time: {:.3f} sec | Elapsed time: {:.3f} sec'.format(
                step, step*dt, elapsed_time(start_time)))
        #if video is None:
        #   time.sleep(sleep_per_step)
        #print(pairwise_collision(robot, door))
        if abs(target_x - point_from_pose(get_link_pose(robot, robot_link))[0]) < 1e-3: # TODO: velocity condition
           break
        # print('Velocities:', get_joint_velocities(robot, robot_joints))
        # #print('Torques:', get_joint_torques(robot, robot_joints))
        # wait_if_gui()
        # print()

    if video is None:
        wait_if_gui('Finish?')
    disconnect()

if __name__ == '__main__':
    main()
