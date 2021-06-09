#!/usr/bin/env python

from __future__ import print_function

import os
import pybullet as p
import time

from pybullet_tools.utils import add_data_path, connect, disconnect, wait_if_gui, set_camera, load_pybullet, \
    HideOutput, draw_global_system, dump_body, get_sample_fn, get_movable_joints, set_joint_positions, \
    enable_gravity, step_simulation, GRAVITY, get_time_step, elapsed_time, control_joints, get_joint_intervals, \
    velocity_control_joints, PI, set_point, Point, STATIC_MASS, NULL_ID, unit_point, unit_quat, get_box_geometry, \
    create_shape, RED, BASE_LINK, set_camera_pose, Pose, get_aabb, approximate_as_prism, get_all_links, get_aabb_center, \
    get_point, get_difference, set_joint_limits, set_collision_margin

# bullet3/examples/pybullet/examples
# experimentalCcdSphereRadius.py
# heightfield.py
# signedDistanceField.py

def create_door(width=0.1, length=1, height=2, mass=1, **kwargs):
    # TODO: frame, hinge, cylinder on end, sliding door, handle, knob
    # TODO: better way of creating these
    geometry = get_box_geometry(width, length, height)
    door_collision, door_visual = create_shape(
        geometry, pose=Pose(Point(y=-length/2., z=height/2.)), **kwargs)
    body = p.createMultiBody(
        baseMass=STATIC_MASS,
        baseCollisionShapeIndex=NULL_ID,
        baseVisualShapeIndex=NULL_ID,
        #basePosition=Point(z=height/2.),
        basePosition=unit_point(),
        baseOrientation=unit_quat(),
        #baseInertialFramePosition=unit_point(),
        #baseInertialFrameOrientation=unit_quat(),
        linkMasses=[mass],
        linkCollisionShapeIndices=[door_collision],
        linkVisualShapeIndices=[door_visual],
        linkPositions=[Point()],
        linkOrientations=[unit_quat()],
        linkInertialFramePositions=[unit_point()],
        linkInertialFrameOrientations=[unit_quat()],
        linkParentIndices=[0],
        linkJointTypes=[p.JOINT_REVOLUTE],
        linkJointAxis=[[0, 0, 1]],
        #physicsClientId=CLIENT,
    )
    set_joint_limits(body, link=0, lower=-PI, upper=PI)
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

        door = load_pybullet('models/door.urdf', fixed_base=True) # From drake
        #set_point(door, Point(z=-.1))
        #door = create_door()

        door_joints = get_movable_joints(door)
        door_links = get_all_links(door)
        aabb = get_aabb(door)
        center = get_aabb_center(aabb)

        print(approximate_as_prism(door, link=door_links[-1]))
        origin = get_point(door) - center
        origin[2] += aabb[0][2]
        print(origin)
        set_point(door, origin)
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