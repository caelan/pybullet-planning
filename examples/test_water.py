#!/usr/bin/env python

from __future__ import print_function

import time
import numpy as np

from pybullet_tools.utils import add_data_path, connect, enable_gravity, user_input, disconnect, create_sphere, set_point, Point, \
    enable_real_time, dump_world, load_model, wait_for_user, set_camera, stable_z, \
    set_color, get_lower_upper, wait_for_duration, simulate_for_duration, load_pybullet


def main():
    connect(use_gui=True)
    add_data_path()

    set_camera(0, -30, 1)
    plane = load_pybullet('plane.urdf', fixed_base=True)
    #plane = load_model('plane.urdf')
    cup = load_model('models/cup.urdf', fixed_base=True)
    #set_point(cup, Point(z=stable_z(cup, plane)))
    set_point(cup, Point(z=.2))
    set_color(cup, (1, 0, 0, .4))

    num_droplets = 100
    #radius = 0.025
    #radius = 0.005
    radius = 0.0025
    # TODO: more efficient ways to make all of these
    droplets = [create_sphere(radius, mass=0.01) for _ in range(num_droplets)] # kg
    cup_thickness = 0.001

    lower, upper = get_lower_upper(cup)
    print(lower, upper)
    buffer = cup_thickness + radius
    lower = np.array(lower) + buffer*np.ones(len(lower))
    upper = np.array(upper) - buffer*np.ones(len(upper))

    limits = zip(lower, upper)
    x_range, y_range = limits[:2]
    z = upper[2] + 0.1
    #x_range = [-1, 1]
    #y_range = [-1, 1]
    #z = 1
    for droplet in droplets:
        x = np.random.uniform(*x_range)
        y = np.random.uniform(*y_range)
        set_point(droplet, Point(x, y, z))

    for i, droplet in enumerate(droplets):
        x, y = np.random.normal(0, 1e-3, 2)
        set_point(droplet, Point(x, y, z+i*(2*radius+1e-3)))

    #dump_world()
    wait_for_user()

    #user_input('Start?')
    enable_gravity()
    simulate_for_duration(5.0)

    # enable_real_time()
    # try:
    #     while True:
    #         enable_gravity() # enable_real_time requires a command
    #         #time.sleep(dt)
    # except KeyboardInterrupt:
    #     pass
    # print()

    #time.sleep(1.0)
    wait_for_user('Finish?')
    disconnect()

if __name__ == '__main__':
    main()