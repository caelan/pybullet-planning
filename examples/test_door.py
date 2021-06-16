#!/usr/bin/env python

from __future__ import print_function

import os
import pybullet as p
import time
import argparse
import numpy as np
import math

from pybullet_tools.utils import add_data_path, connect, disconnect, wait_if_gui, load_pybullet, \
    draw_global_system, dump_body, get_sample_fn, get_movable_joints, enable_gravity, step_simulation, \
    get_time_step, elapsed_time, get_joint_intervals, \
    velocity_control_joints, PI, set_point, Point, get_box_geometry, \
    create_shape, set_camera_pose, Pose, base_aligned_z, BLACK, LinkInfo, create_multi_body, \
    Shape, create_shape_array, unzip, STATIC_MASS, set_position, base_aligned, \
    create_box, BLUE, add_pose_constraint, get_pose, synchronize_viewer, set_renderer, get_cylinder_geometry, \
    wait_for_duration, TURTLEBOT_URDF, set_joint_positions, \
    set_all_color, control_joint, irange, INF, control_joints, get_first_link, point_from_pose, get_link_pose, \
    get_max_velocities, get_max_forces, interpolate, set_joint_position, get_joint_positions, get_closest_points, \
    draw_collision_info, movable_from_joints, compute_jacobian, get_unit_vector, remove_handles, child_link_from_joint, \
    set_configuration, unit_point, get_com_pose, get_link_inertial_pose, draw_pose, tform_point, invert, \
    violates_limits, LockRenderer, get_joint_position, get_base_name, get_body_name
from .test_ramp import condition_controller, simulate


#from examples.test_turtlebot_motion import BASE_JOINTS

# bullet3/examples/pybullet/examples
# experimentalCcdSphereRadius.py
# heightfield.py
# signedDistanceField.py


def create_handle(width, length, height, diameter=0.04):
    handle_height = 5 * diameter
    point = Point(x=width / 2 + diameter / 2 + diameter, y=-3 * length / 4., z=height / 2.)
    shapes = [
        Shape(get_cylinder_geometry(radius=diameter / 2, height=handle_height), color=BLACK),
        Shape(get_cylinder_geometry(radius=diameter / 2, height=diameter),
              pose=Pose(Point(x=-diameter, z=(handle_height - diameter) / 2.)), color=BLACK),
        Shape(get_cylinder_geometry(radius=diameter / 2, height=diameter),
              pose=Pose(Point(x=-diameter, z=-(handle_height - diameter) / 2.)), color=BLACK),
    ]
    handle_collision, handle_visual = create_shape_array(*unzip(shapes))
    return LinkInfo(mass=1, collision_id=handle_collision, visual_id=handle_visual,
                    point=point, parent=1, joint_type=p.JOINT_FIXED)

def create_frame(width, length, height, side=0.1):
    # TODO: could be part of the base link instead
    shapes = [
        Shape(get_box_geometry(width=width, length=length + 2 * side, height=side),
              pose=Pose(Point(z=height / 2. + side / 2.)), color=BLACK),
        Shape(get_box_geometry(width=width, length=side, height=height),
              pose=Pose(Point(y=(length + side) / 2.)), color=BLACK),
        Shape(get_box_geometry(width=width, length=side, height=height),
              pose=Pose(Point(y=-(length + side) / 2.)), color=BLACK),
    ]
    frame_collision, frame_visual = create_shape_array(*unzip(shapes))
    return LinkInfo(mass=STATIC_MASS, collision_id=frame_collision, visual_id=frame_visual,
                    point=Point(y=-length / 2., z=height / 2.),
                    parent=0, joint_type=p.JOINT_FIXED)

def create_door(width=0.08, length=1, height=2, mass=1, handle=True, frame=True, **kwargs):
    # TODO: hinge, cylinder on end, sliding door, knob
    # TODO: explicitly disable self collisions (happens automatically)
    geometry = get_box_geometry(width, length, height)
    hinge = 0 # -width/2
    com_point = Point(x=hinge, y=-length/2., z=height/2.)
    door_collision, door_visual = create_shape(
        geometry, pose=Pose(com_point),
        **kwargs)
    door_link = LinkInfo(mass=mass, collision_id=door_collision, visual_id=door_visual,
                         #point=com_point,
                         inertial_point=com_point, # TODO: be careful about the COM
                         parent=0, joint_type=p.JOINT_REVOLUTE, joint_axis=[0, 0, 1])
    links = [door_link]
    if handle:
        links.append(create_handle(width, length, height))
    if frame:
        links.append(create_frame(width, length, height))
    body = create_multi_body(base_link=LinkInfo(), links=links)
    #draw_circle(center=unit_point(), diameter=width/2., parent=body, parent_link=0)
    #set_joint_limits(body, link=0, lower=-PI, upper=PI)
    return body

##################################################

def test_door(door):
    door_joints = get_movable_joints(door)

    sample_fn = get_sample_fn(door, door_joints)
    set_joint_positions(door, door_joints, sample_fn())
    while True:
        positions = sample_fn()
        set_joint_positions(door, door_joints, positions)
        wait_if_gui()

    lower, upper = get_joint_intervals(door, door_joints)
    control_joints(door, door_joints, positions=lower)
    velocity_control_joints(door, door_joints, velocities=[PI / 4]) # Able to exceed limits

##################################################

IGNORE_EXT = ['.png', '.gif', '.jpeg', '.py', '.mtl']

def list_pybullet_robots():
    data_path = add_data_path()
    robot_path = os.path.abspath(os.path.join(data_path, os.pardir, 'pybullet_robots'))
    directories = sorted(name for name in os.listdir(robot_path) if os.path.isdir(os.path.join(robot_path, name)))
    return directories

def list_pybullet_data():
    # TODO: recursively search
    data_path = add_data_path()
    #directories = sorted(name for name in os.listdir(data_path) if os.path.isdir(os.path.join(data_path, name)))
    files = sorted(name for name in os.listdir(data_path) if not os.path.isdir(
        os.path.join(data_path, name)) and (os.path.splitext(name)[1] not in IGNORE_EXT))
    return files

def load_plane(z=-1e-3):
    add_data_path()
    plane = load_pybullet('plane.urdf', fixed_base=True)
    #plane = load_model('plane.urdf')
    if z is not None:
        set_point(plane, Point(z=z))
    return plane

##################################################

def solve_collision_free(door, obstacle, max_iterations=100, step_size=math.radians(5), min_distance=2e-2, draw=True):
    joints = get_movable_joints(door)
    door_link = child_link_from_joint(joints[-1])

    # print(get_com_pose(door, door_link))
    # print(get_link_inertial_pose(door, door_link))
    # print(get_link_pose(door, door_link))
    # draw_pose(get_com_pose(door, door_link))

    handles = []
    success = False
    start_time = time.time()
    for iteration in range(max_iterations):
        current_conf = np.array(get_joint_positions(door, joints))
        collision_infos = get_closest_points(door, obstacle, link1=door_link, max_distance=min_distance)
        if not collision_infos:
            success = True
            break
        collision_infos = sorted(collision_infos, key=lambda info: info.contactDistance)
        collision_infos = collision_infos[:1] # TODO: average all these
        if draw:
            for collision_info in collision_infos:
                handles.extend(draw_collision_info(collision_info))
            wait_if_gui()
        [collision_info] = collision_infos[:1]
        distance = collision_info.contactDistance
        print('Iteration: {} | Collisions: {} | Distance: {:.3f} | Time: {:.3f}'.format(
           iteration, len(collision_infos), distance, elapsed_time(start_time)))
        if distance >= min_distance:
            success = True
            break
        # TODO: convergence or decay in step size
        direction = step_size * get_unit_vector(collision_info.contactNormalOnB) # B->A (already normalized)
        contact_point = collision_info.positionOnA
        #com_pose = get_com_pose(door, door_link) # TODO: be careful here
        com_pose = get_link_pose(door, door_link)
        local_point = tform_point(invert(com_pose), contact_point)
        #local_point = unit_point()

        translate, rotate = compute_jacobian(door, door_link, point=local_point)
        delta_conf = np.array([np.dot(translate[mj], direction)  # + np.dot(rotate[mj], direction)
                               for mj in movable_from_joints(door, joints)])
        new_conf = current_conf + delta_conf
        if violates_limits(door, joints, new_conf):
            break
        set_joint_positions(door, joints, new_conf)
        if draw:
            wait_if_gui()
    remove_handles(handles)
    print('Success: {} | Iteration: {} | Time: {:.3f}'.format(
        success, iteration, elapsed_time(start_time)))
    #quit()
    return success

def test_kinematic(robot, door, target_x):
    wait_if_gui('Begin?')
    robot_joints = get_movable_joints(robot)[:3]
    joint = robot_joints[0]
    start_x = get_joint_position(robot, joint)
    num_steps = int(math.ceil(abs(target_x - start_x) / 1e-2))
    for x in interpolate(start_x, target_x, num_steps=num_steps):
        set_joint_position(robot, joint=joint, value=x)
        #with LockRenderer():
        solve_collision_free(door, robot, draw=False)
        wait_for_duration(duration=1e-2)
        #wait_if_gui()
    wait_if_gui('Finish?')

def test_simulation(robot, target_x, video=None):
    use_turtlebot = (get_body_name(robot) == 'turtlebot')
    if not use_turtlebot:
        target_point, target_quat = map(list, get_pose(robot))
        target_point[0] = target_x
        add_pose_constraint(robot, pose=(target_point, target_quat), max_force=200)  # TODO: velocity constraint?
    else:
        # p.changeDynamics(robot, robot_joints[0], # Doesn't work
        #                  maxJointVelocity=1,
        #                  jointLimitForce=1,)
        robot_joints = get_movable_joints(robot)[:3]
        print('Max velocities:', get_max_velocities(robot, robot_joints))
        print('Max forces:', get_max_forces(robot, robot_joints))
        control_joint(robot, joint=robot_joints[0], position=target_x, velocity=0,
                      position_gain=None, velocity_scale=None, max_velocity=100, max_force=300)
        # control_joints(robot, robot_joints, positions=[target_x, 0, PI], max_force=300)
        # velocity_control_joints(robot, robot_joints, velocities=[-2., 0, 0]) #, max_force=300)

    robot_link = get_first_link(robot)
    if video is None:
        wait_if_gui('Begin?')
    simulate(controller=condition_controller(
        lambda *args: abs(target_x - point_from_pose(get_link_pose(robot, robot_link))[0]) < 1e-3), sleep=0.01) # TODO: velocity condition
    # print('Velocities:', get_joint_velocities(robot, robot_joints))
    # print('Torques:', get_joint_torques(robot, robot_joints))
    if video is None:
        set_renderer(enable=True)
        wait_if_gui('Finish?')

##################################################

def main(use_turtlebot=True):
    parser = argparse.ArgumentParser()
    parser.add_argument('-sim', action='store_true')
    parser.add_argument('-video', action='store_true')
    args = parser.parse_args()
    video = 'video.mp4' if args.video else None

    connect(use_gui=True, mp4=video)
    #set_renderer(enable=False)
    # print(list_pybullet_data())
    # print(list_pybullet_robots())

    draw_global_system()
    set_camera_pose(camera_point=Point(+1.5, -1.5, +1.5),
                    target_point=Point(-1.5, +1.5, 0))

    plane = load_plane()
    #door = load_pybullet('models/door.urdf', fixed_base=True) # From drake
    #set_point(door, Point(z=-.1))
    door = create_door()
    #set_position(door, z=base_aligned_z(door))
    set_point(door, base_aligned(door))
    #set_collision_margin(door, link=0, margin=0.)
    set_configuration(door, [math.radians(-5)])
    dump_body(door)

    door_joint = get_movable_joints(door)[0]
    door_link = child_link_from_joint(door_joint)
    #draw_pose(get_com_pose(door, door_link), parent=door, parent_link=door_link)
    draw_pose(Pose(), parent=door, parent_link=door_link)
    wait_if_gui()

    ##########

    start_x = +2
    target_x = -start_x
    if not use_turtlebot:
        side = 0.25
        robot = create_box(w=side, l=side, h=side, mass=5., color=BLUE)
        set_position(robot, x=start_x)
        #set_velocity(robot, linear=Point(x=-1))
    else:
        turtlebot_urdf = os.path.abspath(TURTLEBOT_URDF)
        print(turtlebot_urdf)
        #print(read(turtlebot_urdf))
        robot = load_pybullet(turtlebot_urdf, merge=True, fixed_base=True)
        robot_joints = get_movable_joints(robot)[:3]
        set_joint_positions(robot, robot_joints, [start_x, 0, PI])
    set_all_color(robot, BLUE)
    set_position(robot, z=base_aligned_z(robot))
    dump_body(robot)

    ##########

    set_renderer(enable=True)
    #test_door(door)
    if args.sim:
        test_simulation(robot, target_x, video)
    else:
        assert use_turtlebot # TODO: extend to the block
        test_kinematic(robot, door, target_x)
    disconnect()

if __name__ == '__main__':
    main()
