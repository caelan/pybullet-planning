#!/usr/bin/env python

from __future__ import print_function

import os
import pybullet as p
import time

from pybullet_tools.utils import add_data_path, connect, disconnect, wait_if_gui, set_camera, load_pybullet, \
    HideOutput, draw_global_system, dump_body, get_sample_fn, get_movable_joints, set_joint_positions, \
    enable_gravity, step_simulation, GRAVITY, get_time_step, elapsed_time, control_joints, get_joint_intervals, \
    velocity_control_joints, PI, set_point, Point, STATIC_MASS, NULL_ID, unit_point, unit_quat, get_box_geometry, \
    create_shape, RED, BASE_LINK, set_camera_pose

# bullet3/examples/pybullet/examples
# experimentalCcdSphereRadius.py
# heightfield.py
# signedDistanceField.py

def create_door(width=1, length=0.1, height=2, mass=1, **kwargs):
    geometry = get_box_geometry(width, length, height)
    collision_id, visual_id = create_shape(geometry, **kwargs)
    body = p.createMultiBody(
        baseMass=STATIC_MASS,
        baseCollisionShapeIndex=NULL_ID,
        baseVisualShapeIndex=NULL_ID,
        basePosition=Point(z=height/2.),
        baseOrientation=unit_quat(),
        #baseInertialFramePosition=unit_point(),
        #baseInertialFrameOrientation=unit_quat(),
        linkMasses=[mass],
        linkCollisionShapeIndices=[collision_id],
        linkVisualShapeIndices=[visual_id],
        linkPositions=[unit_point()],
        linkOrientations=[unit_quat()],
        linkInertialFramePositions=[unit_point()],
        linkInertialFrameOrientations=[unit_quat()],
        linkParentIndices=[0],
        linkJointTypes=[p.JOINT_REVOLUTE],
        linkJointAxis=[[0, 0, 1]],
        #physicsClientId=CLIENT,
    )
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
        #door = load_pybullet('models/door.urdf', fixed_base=True) # From drake
        #set_point(door, Point(z=-.1))

        door = create_door()
        door_joints = get_movable_joints(door)

    dump_body(door)
    sample_fn = get_sample_fn(door, door_joints)
    set_joint_positions(door, door_joints, sample_fn())
    # while True:
    #     positions = sample_fn()
    #     set_joint_positions(door, door_joints, positions)
    #     wait_if_gui()

    lower, upper = get_joint_intervals(door, door_joints)
    #control_joints(door, door_joints, positions=lower)
    velocity_control_joints(door, door_joints, velocities=[PI / 4]) # Able to exceed limits

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