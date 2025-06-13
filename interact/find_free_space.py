#!/usr/bin/env python3
"""Code to relocate one object to free space. Cannot track multiple changes."""

from __future__ import print_function

from pybullet_tools.franka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, \
    get_ik_fn, get_free_motion_gen, get_holding_motion_gen, get_grasping_fn, BodyGrasp
from pybullet_tools.utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, \
    draw_global_system, Pose, Point, set_default_camera, BLOCK_URDF, load_model, wait_if_gui, \
    disconnect, DRAKE_IIWA_URDF, update_state, disable_real_time, HideOutput, load_pybullet, \
    get_movable_joints, set_joint_positions, create_box, RGBA, get_point, get_euler, refine_path, \
    inverse_kinematics, end_effector_from_body, get_pose, link_from_name, get_link_pose, invert, set_camera_pose

from pybullet_tools.ikfast.franka_panda.ik import FRANKA_URDF, PANDA_INFO

import pybullet as p
import numpy as np
import argparse
import os
from glob import glob
import random

from scipy.spatial.transform import Rotation as R


def find_delta_pose(scene_mesh, new_mesh, max_dist):
    """Function to find the delta position of the new object.
    Find free space the object can be situated in"""
    default_pos, default_ori = p.getBasePositionAndOrientation(new_mesh)
    found_place = False
    while not found_place:
        x_perturb = random.uniform(-max_dist, max_dist)
        y_perturb = random.uniform(-max_dist, max_dist)
        # move our mesh
        p.resetBasePositionAndOrientation(new_mesh,
                                          [x_perturb + default_pos[0], y_perturb + default_pos[1], default_pos[2]],
                                          default_ori)

        # update and step our simulation
        p.stepSimulation()
        # calculate collision
        pts = p.getContactPoints()
        print("Contact_points: {}".format(len(pts)))
        if len(pts) == 0:
            # # found place without contact
            breakpoint()
            return [x_perturb, y_perturb, 0]


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path", help='path to cluster dir')
    parser.add_argument("--cluster_idx", type=int, default=0, help='the index of the mesh to move')
    parser.add_argument("--max_dist", type=float, default=0.15, help='maximum deviation of the mesh (x, y)')
    args = parser.parse_args()
    max_dist = args.max_dist

    # load scene meshes and set up scene
    connect(use_gui=True)
    # p.connect(p.GUI)
    disable_real_time()
    # add_data_path()
    draw_global_system()
    # with HideOutput():
        # robot = load_model(FRANKA_URDF) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        # robot = load_pybullet(FRANKA_URDF, fixed_base=True)
        # robot = load_pybullet(FRANKA_URDF.replace('panda_arm_hand', 'panda_arm_hand_cam_notip'), fixed_base=True)
        # floor = p.loadURDF('plane.urdf')
        # floor = load_model('models/short_floor.urdf')
        # set_pose(floor, Pose(Point(x=1.2,  z=-0.02)))
    set_camera_pose(camera_point=[0.8, -0.2, 0.8])
    # conf = (0, -np.pi / 4.0, 0, -3.0 * np.pi / 4.0, 0, np.pi / 2, np.pi / 4, 0.04, 0.04)
    # # move robot to start position
    # joints = get_movable_joints(robot)
    # set_joint_positions(robot, joints, conf)
    saved_world = WorldSaver()

    # Load the scene broken down in convex
    scene_mesh = []
    # get the upper most layer clusters
    upper_nodes = np.loadtxt(os.path.join(args.path, "upper_nodes.txt")).astype(np.uint8).tolist()
    for i in upper_nodes:
        fname = os.path.join(args.path, "mesh_{:02d}.urdf".format(i))
        mesh = p.loadURDF(fname)
        scene_mesh.append(mesh)

    # get the mesh to move
    mesh_move = p.loadURDF(os.path.join(args.path, "mesh_{:02d}.urdf".format(args.cluster_idx)))

    # find new position to put our object
    default_pos, default_ori = p.getBasePositionAndOrientation(mesh_move)
    found_place = False
    idx = 0
    while not found_place:
        x_perturb = random.uniform(-max_dist, max_dist)
        y_perturb = random.uniform(-max_dist, max_dist)
        # move our mesh
        p.resetBasePositionAndOrientation(mesh_move,
                                          [x_perturb + default_pos[0], y_perturb + default_pos[1], default_pos[2]],
                                          default_ori)

        # update and step our simulation
        # calculate collision
        # Run the simulation for a few steps
        for _ in range(100):
            p.stepSimulation()
        min_dist = 10
        for mesh in scene_mesh: 
            dist = p.getClosestPoints(mesh, mesh_move, distance=10)
            if dist[0][8] < min_dist:
                min_dist = dist[0][8]
        print(min_dist)
        if min_dist > 0.01:
            found_place = True

    # save delta pos to disk
    np.savetxt(os.path.join(args.path, "delta_pos_{}.txt".format(args.cluster_idx)),
               np.array([x_perturb, y_perturb, 0]))
    print("======================= Results =====================")
    print("x perturb : {}".format(x_perturb))
    print("y perturb : {}".format(y_perturb))

