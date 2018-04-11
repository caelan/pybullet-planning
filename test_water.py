#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import time
import numpy as np

from pr2_utils import TOP_HOLDING_LEFT_ARM, ARM_JOINT_NAMES, TORSO_JOINT_NAME, \
    REST_RIGHT_ARM, SIDE_HOLDING_LEFT_ARM, BASE_JOINT_NAMES
from pybullet_utils import set_base_values, joint_from_name, set_joint_position, \
    set_joint_positions, add_data_path, connect, plan_base_motion, plan_joint_motion, enable_gravity, input, \
    joint_controller, disconnect, wait_for_interrupt, create_sphere, set_point, Point, \
    enable_real_time, wait_for_duration, dump_world


def main():
    connect(use_gui=True)
    add_data_path()

    plane = p.loadURDF("plane.urdf")
    #table = p.loadURDF("table/table.urdf", 0, 0, 0, 0, 0, 0.707107, 0.707107)
    #table = p.loadURDF("table_square/table_square.urdf")
    #table = p.loadURDF("cube.urdf")
    #table = p.loadURDF("block.urdf")
    #table = p.loadURDF("door.urdf")

    num_droplets = 100
    radius = 0.025
    droplets = [create_sphere(radius, mass=1) for _ in range(num_droplets)]

    x_range = [-1, 1]
    y_range = [-1, 1]
    z = 1
    for droplet in droplets:
        x = np.random.uniform(*x_range)
        y = np.random.uniform(*y_range)
        set_point(droplet, Point(x, y, z))
    dump_world()

    input('Start?')
    dt = 1. / 240
    #dt = 0
    print('dt:', dt)
    #wait_for_duration(5.0, dt=dt)

    enable_real_time()
    enable_gravity()
    try:
        while True:
            enable_gravity() # enable_real_time requires a command
            #time.sleep(dt)
    except KeyboardInterrupt:
        pass
    print()


    #enable_real_time()
    #time.sleep(1.0)
    #wait_for_interrupt()
    input('Finish?')
    disconnect()



if __name__ == '__main__':
    main()