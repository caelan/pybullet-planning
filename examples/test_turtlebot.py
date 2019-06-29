#!/usr/bin/env python

from __future__ import print_function

import random

import numpy as np
from pybullet_tools.utils import connect, load_model, disconnect, wait_for_user, create_box, set_point, dump_body, \
    TURTLEBOT_URDF, HideOutput, LockRenderer, joint_from_name, set_euler, get_euler, get_point, \
    set_joint_position, get_joint_positions, pairwise_collision, stable_z, wait_for_duration, get_link_pose, \
    link_from_name, get_pose, euler_from_quat, multiply, invert, draw_pose, unit_point, unit_quat, \
    remove_debug, get_aabb, draw_aabb, get_subtree_aabb, ROOMBA_URDF, set_all_static, user_input, threaded_input

# RGBA colors (alpha is transparency)
RED = (1, 0, 0, 1)
TAN = (0.824, 0.706, 0.549, 1)

def main(floor_width=2.0):
    # Creates a pybullet world and a visualizer for it
    connect(use_gui=True)
    identity_pose = (unit_point(), unit_quat())
    origin_handles = draw_pose(identity_pose, length=1.0) # Draws the origin coordinate system (x:RED, y:GREEN, z:BLUE)

    # Bodies are described by an integer index
    floor = create_box(w=floor_width, l=floor_width, h=0.001, color=TAN) # Creates a tan box object for the floor
    set_point(floor, [0, 0, -0.001 / 2.]) # Sets the [x,y,z] translation of the floor

    obstacle = create_box(w=0.5, l=0.5, h=0.1, color=RED) # Creates a red box obstacle
    set_point(obstacle, [0.5, 0.5, 0.1 / 2.]) # Sets the [x,y,z] position of the obstacle
    print('Position:', get_point(obstacle))
    set_euler(obstacle, [0, 0, np.pi / 4]) #  Sets the [roll,pitch,yaw] orientation of the obstacle
    print('Orientation:', get_euler(obstacle))

    with LockRenderer(): # Temporarily prevents the renderer from updating for improved loading efficiency
        with HideOutput(): # Temporarily suppresses pybullet output
            robot = load_model(ROOMBA_URDF) # Loads a robot from a *.urdf file
            robot_z = stable_z(robot, floor) # Returns the z offset required for robot to be placed on floor
            set_point(robot, [0, 0, robot_z]) # Sets the z position of the robot
    dump_body(robot) # Prints joint and link information about robot
    set_all_static()

    # Joints are also described by an integer index
    # The turtlebot has explicit joints representing x, y, theta
    x_joint = joint_from_name(robot, 'x') # Looks up the robot joint named 'x'
    y_joint = joint_from_name(robot, 'y') # Looks up the robot joint named 'y'
    theta_joint = joint_from_name(robot, 'theta') # Looks up the robot joint named 'theta'
    joints = [x_joint, y_joint, theta_joint]

    base_link = link_from_name(robot, 'base_link') # Looks up the robot link named 'base_link'
    world_from_obstacle = get_pose(obstacle) # Returns the pose of the origin of obstacle wrt the world frame
    obstacle_aabb = get_subtree_aabb(obstacle)
    draw_aabb(obstacle_aabb)

    random.seed(0) # Sets the random number generator state
    handles = []
    for i in range(10):
        for handle in handles:
            remove_debug(handle)
        print('\nIteration: {}'.format(i))
        x = random.uniform(-floor_width/2., floor_width/2.)
        set_joint_position(robot, x_joint, x) # Sets the current value of the x joint
        y = random.uniform(-floor_width/2., floor_width/2.)
        set_joint_position(robot, y_joint, y) # Sets the current value of the y joint
        yaw = random.uniform(-np.pi, np.pi)
        set_joint_position(robot, theta_joint, yaw) # Sets the current value of the theta joint
        values = get_joint_positions(robot, joints) # Obtains the current values for the specified joints
        print('Joint values: [x={:.3f}, y={:.3f}, yaw={:.3f}]'.format(*values))

        world_from_robot = get_link_pose(robot, base_link) # Returns the pose of base_link wrt the world frame
        position, quaternion = world_from_robot # Decomposing pose into position and orientation (quaternion)
        x, y, z = position # Decomposing position into x, y, z
        print('Base link position: [x={:.3f}, y={:.3f}, z={:.3f}]'.format(x, y, z))
        euler = euler_from_quat(quaternion) # Converting from quaternion to euler angles
        roll, pitch, yaw = euler # Decomposing orientation into roll, pitch, yaw
        print('Base link orientation: [roll={:.3f}, pitch={:.3f}, yaw={:.3f}]'.format(roll, pitch, yaw))
        handles.extend(draw_pose(world_from_robot, length=0.5)) # # Draws the base coordinate system (x:RED, y:GREEN, z:BLUE)
        obstacle_from_robot = multiply(invert(world_from_obstacle), world_from_robot) # Relative transformation from robot to obstacle

        robot_aabb = get_subtree_aabb(robot, base_link) # Computes the robot's axis-aligned bounding box (AABB)
        lower, upper = robot_aabb # Decomposing the AABB into the lower and upper extrema
        center = (lower + upper)/2. # Computing the center of the AABB
        extent = upper - lower # Computing the dimensions of the AABB
        handles.extend(draw_aabb(robot_aabb))

        collision = pairwise_collision(robot, obstacle) # Checks whether robot is currently colliding with obstacle
        print('Collision: {}'.format(collision))
        wait_for_duration(1.0) # Like sleep() but also updates the viewer
    wait_for_user() # Like raw_input() but also updates the viewer

    # Destroys the pybullet world
    disconnect()

if __name__ == '__main__':
    main()
