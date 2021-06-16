#!/usr/bin/env python

from __future__ import print_function

import argparse
import time
import numpy as np

from itertools import product

from pybullet_tools.utils import add_data_path, connect, disconnect, wait_if_gui, load_pybullet, \
    draw_global_system, dump_body, enable_gravity, step_simulation, \
    get_time_step, elapsed_time, set_point, Point, set_camera_pose, set_position, create_box, BLUE, synchronize_viewer, \
    set_renderer, irange, INF, create_cylinder, create_sphere, create_capsule, set_euler, get_velocity, create_faces, \
    STATIC_MASS, mesh_from_points, RED, wait_for_duration

def create_ramp(half_extent=0.5):
    vertices = [Point(s1 * half_extent, s2 * half_extent, 0) for s1, s2 in product([+1, -1], repeat=2)] + \
               [Point(-half_extent, s * half_extent, half_extent) for s in [+1, -1]]
    mesh = mesh_from_points(vertices, under=True)
    return create_faces(mesh, mass=STATIC_MASS, color=RED)

def create_object(shape, side=0.1, mass=1, color=BLUE):
    # TODO: simulation parameters
    # TODO: object dynamics parameters
    if shape == 'box':
        obj = create_box(w=side, l=side, h=side, mass=mass, color=color)
    elif shape == 'sphere':
        obj = create_sphere(radius=side, mass=mass, color=color)
    elif shape == 'cylinder':
        obj = create_cylinder(radius=side, height=side, mass=mass, color=color)
    elif shape == 'capsule':
        obj = create_capsule(radius=side, height=side, mass=mass, color=color)
    else:
        raise ValueError(shape)
    return obj

##################################################

def condition_controller(condition):
    #from itertools import takewhile
    dt = get_time_step()
    for step in irange(INF):
        #duration = step*dt
        if condition(step):
           break
        yield

def at_rest(obj, tol=1e-3):
    linear, angular = get_velocity(obj)
    return np.allclose(linear, 0, rtol=0, atol=tol) and \
           np.allclose(angular, 0, rtol=0, atol=tol)

def is_empty(generator):
    try:
        next(generator)
        return False
    except StopIteration:
        return True

def simulate(controller=None, max_duration=INF, max_steps=INF, print_rate=1., sleep=None):
    enable_gravity()
    dt = get_time_step()
    print('Time step: {:.6f} sec'.format(dt))
    start_time = last_print = time.time()
    for step in irange(max_steps):
        duration = step * dt
        if duration >= max_duration:
            break
        if (controller is not None) and is_empty(controller):
            break
        step_simulation()
        synchronize_viewer()
        if elapsed_time(last_print) >= print_rate:
            print('Sim step: {} | Sim time: {:.3f} sec | Elapsed time: {:.3f} sec'.format(
                step, duration, elapsed_time(start_time)))
            last_print = time.time()
        if sleep is not None:
            time.sleep(sleep)
            #wait_for_duration(sleep)
            #wait_if_gui()

##################################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-shape', default='box', choices=['box', 'sphere', 'cylinder', 'capsule'])
    parser.add_argument('-video', action='store_true')
    args = parser.parse_args()
    video = 'video.mp4' if args.video else None

    connect(use_gui=True, mp4=video)
    if video is not None:
        set_renderer(enable=False)

    draw_global_system()
    set_camera_pose(camera_point=Point(+1.5, -1.5, +1.5),
                    target_point=Point(-1.5, +1.5, 0))

    add_data_path()
    plane = load_pybullet('plane.urdf', fixed_base=True)
    #plane = load_model('plane.urdf')
    set_point(plane, Point(z=-1e-3))

    ramp = create_ramp()
    dump_body(ramp)

    obj = create_object(args.shape)
    set_euler(obj, np.random.uniform(-np.math.radians(1), np.math.radians(1), 3))
    set_point(obj, np.random.uniform(-1e-3, +1e-3, 3))
    #set_velocity(obj, linear=Point(x=-1))
    set_position(obj, z=2.)
    #set_position(obj, z=base_aligned_z(obj))
    dump_body(obj)

    #add_pose_constraint(obj, pose=(target_point, target_quat), max_force=200)

    set_renderer(enable=True)
    if video is None:
        wait_if_gui('Begin?')
    simulate(controller=condition_controller(lambda step: (step != 0) and at_rest(obj)))
    if video is None:
        wait_if_gui('Finish?')
    disconnect()

if __name__ == '__main__':
    main()
