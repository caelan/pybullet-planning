#!/usr/bin/env python

from __future__ import print_function

import pybullet as p

from pybullet_utils import connect, input, dump_world, set_pose, Pose, Point, set_default_camera, stable_z, \
    KUKA_IIWA_URDF, BLOCK_URDF, \
    SINK_URDF, STOVE_URDF, load_model, wait_for_interrupt, wait


def main():
    connect(use_gui=True)

    #print(get_data_path())
    #p.loadURDF("samurai.urdf", useFixedBase=True) # World
    #p.loadURDF("kuka_lwr/kuka.urdf", useFixedBase=True)
    #p.loadURDF("kuka_iiwa/model_free_base.urdf", useFixedBase=True)

    robot = load_model(KUKA_IIWA_URDF)
    floor = load_model('models/short_floor.urdf')
    block = load_model(BLOCK_URDF)
    sink = load_model(SINK_URDF, pose=Pose(Point(x=-0.5)))
    stove = load_model(STOVE_URDF, pose=Pose(Point(x=+0.5)))

    print(stable_z(block, floor))
    set_pose(block, Pose(Point(y=0.5, z=stable_z(block, floor))))

    #print(get_camera())
    set_default_camera()
    #print(get_camera())

    dump_world()

    wait_for_interrupt()
    #wait(5)
    input('Finish?')
    p.disconnect()

if __name__ == '__main__':
    main()