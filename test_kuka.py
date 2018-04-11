#!/usr/bin/env python

from __future__ import print_function

import pybullet as p
import time
import numpy as np

from pybullet_utils import set_base_values, joint_from_name, set_joint_position, \
    set_joint_positions, add_data_path, connect, plan_base_motion, plan_joint_motion, control_joints, \
    enable_gravity, get_joint_positions, is_circular, get_joint_limits, get_data_path, input, step_simulation, \
    dump_world, set_pose, Pose, Point, Euler, get_camera, set_default_camera, stable_z
from pr2_utils import TOP_HOLDING_LEFT_ARM, ARM_JOINT_NAMES, TORSO_JOINT_NAME, \
    REST_RIGHT_ARM, SIDE_HOLDING_LEFT_ARM, set_arm_conf, BASE_JOINT_NAMES

KUKA_IIWA_URDF = "kuka_iiwa/model.urdf"
KUKA_IIWA_GRIPPER_SDF = "kuka_iiwa/kuka_with_gripper.sdf"

R2D2_URDF = "r2d2.urdf"
MINITAUR_URDF = "quadruped/minitaur.urdf"
HUMANOID_MJCF = "mjcf/humanoid.xml"
HUSKY_URDF = "husky/husky.urdf"

KIVA_SHELF_SDF = "kiva_shelf/model.sdf"

#BLOCK_URDF = "models/drake/objects/block_for_pick_and_place.urdf"
BLOCK_URDF = "models/drake/objects/block_for_pick_and_place_mid_size.urdf"

def load_model(model_file, pose=None, fixed_base=True):
    add_data_path()
    if model_file.endswith('.urdf'):
        body = p.loadURDF(model_file, useFixedBase=fixed_base)
    elif model_file.endswith('.sdf'):
        body = p.loadSDF(model_file)
    elif model_file.endswith('.xml'):
        body = p.loadMJCF(model_file)
    elif model_file.endswith('.bullet'):
        body = p.loadBullet(model_file)
    else:
        raise ValueError(model_file)
    if pose is not None:
        set_pose(body, pose)
    return body

def examine():
    print('Press Ctrl-C to continue')
    try:
        while True:
            step_simulation()
    except KeyboardInterrupt:
        pass
    finally:
        print()

def main():
    connect(use_gui=True)

    #print(get_data_path())
    #p.loadURDF("r2d2.urdf", useFixedBase=True)
    #p.loadURDF("samurai.urdf", useFixedBase=True) # World
    #p.loadURDF("quadruped/minitaur.urdf", useFixedBase=True) # Walker
    #p.loadURDF("kuka_lwr/kuka.urdf", useFixedBase=True)
    #p.loadURDF("kuka_iiwa/model.urdf", useFixedBase=True)
    #p.loadURDF("kuka_iiwa/model_free_base.urdf", useFixedBase=True)
    #p.loadSDF("kuka_iiwa/kuka_with_gripper.sdf")
    #p.loadMJCF("kuka_iiwa/kuka_with_gripper.sdf", useFixedBase=True)
    #p.loadMJCF("mjcf/humanoid.xml")
    #p.loadBullet(...)
    #p.loadURDF("husky/husky.urdf", useFixedBase=True) # Car
    #p.loadURDF("kiva_shelf/model.sdf", useFixedBase=True) # Car

    robot = load_model(KUKA_IIWA_URDF)
    floor = load_model('models/short_floor.urdf')
    block = load_model(BLOCK_URDF)
    print(stable_z(block, floor))
    set_pose(block, Pose(Point(y=0.5, z=stable_z(block, floor))))

    #print(get_camera())
    set_default_camera()
    #print(get_camera())


    dump_world()

    examine()
    input('Finish?')
    p.disconnect()

if __name__ == '__main__':
    main()