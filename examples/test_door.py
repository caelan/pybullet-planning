#!/usr/bin/env python

from __future__ import print_function

from pybullet_tools.utils import add_data_path, connect, disconnect, wait_if_gui, set_camera, load_pybullet, \
    HideOutput, draw_global_system, dump_body, get_sample_fn, get_movable_joints, set_joint_positions


def main():
    connect(use_gui=True)
    print(add_data_path())
    draw_global_system()
    set_camera(0, -30, 1)
    with HideOutput():
        plane = load_pybullet('plane.urdf', fixed_base=True)
        #plane = load_model('plane.urdf')
        door = load_pybullet('models/door.urdf', fixed_base=True)
        door_joints = get_movable_joints(door)

    dump_body(door)
    sample_fn = get_sample_fn(door, door_joints)
    while True:
        positions = sample_fn()
        set_joint_positions(door, door_joints, positions)
        wait_if_gui()

    wait_if_gui('Finish?')
    disconnect()

if __name__ == '__main__':
    main()