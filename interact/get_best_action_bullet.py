#!/usr/bin/env python3
"""Code to sample and evalute multiple possible actions. Input is the Instance candidate tree and its visiblity. Output the best action accordinge to reward function. Deprecated version"""

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
import pybullet_data
import numpy as np
import argparse
import os
import json
from glob import glob
import random
import pickle
import matplotlib.pyplot as plt
import open3d as o3d
from tqdm import tqdm
import open3d as o3d
import importlib.util

from scipy.spatial.transform import Rotation as R
from interact.interact_utils import Trie, Node


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

def load_tree(path):
    """Function to load the tree and uncertainties"""
    tree_path = os.path.join(os.path.dirname(path), 'tree.pkl')
    with open(tree_path, "rb") as f:
        tree = pickle.load(f)
    # first get the leaf nodes
    leaf_nodes = tree.get_leaf_nodes()
    leaf_node_idxs = []
    mesh_paths = []
    for leaf_node in leaf_nodes:
        # keep leaf node idx
        leaf_node_idxs.append(leaf_node.key)
        # load the cluster
        mesh_paths.append(os.path.join(path, "mesh_{:02d}.urdf".format(leaf_node.key)))
    return leaf_node_idxs, mesh_paths

def set_camera_poses(args, meshes):
    """Function to obtain and apply the camera poses that look at the mesh from reasonable distance"""
    # let's first get the bounding box of all meshes
    aabb_mins = []
    aabb_maxs = []
    for body_idx in meshes:
        aabb_min, aabb_max = p.getAABB(body_idx)
        aabb_mins.append(aabb_min)
        aabb_maxs.append(aabb_max)
    aabb_mins_np = np.array(aabb_mins)
    aabb_maxs_np = np.array(aabb_maxs)
    total_aabb_min = aabb_mins_np.min(0)
    total_aabb_max = aabb_maxs_np.max(0)

    center = (total_aabb_min + total_aabb_max) / 2

    # the GUI camera
    p.resetDebugVisualizerCamera(
        cameraDistance=1.5,
        cameraYaw=180,
        cameraPitch=-65,
        cameraTargetPosition=[center[0], center[1], center[2]]
    )
    # the render poses
    cam_poses = []
    for theta in np.arange(0, 2 * np.pi, 2 * np.pi / 10):
        x = 1.0 * np.cos(theta)
        y = 1.0 * np.sin(theta)
        z = 1.0
        cam_poses.append(([center[0] + x, center[1] + y, center[2] + z], [center[0], center[1], center[2]]))
  
    return cam_poses, center, total_aabb_min[-1]


def load_mesh_with_mtl(obj_path):
    # Ensure textures can be found
    mesh = o3d.io.read_triangle_mesh(obj_path, enable_post_processing=True)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    return mesh

def apply_translation(mesh, translation):
    mesh.translate(translation)
    return mesh

def render_view(mesh, intrinsic, extrinsic, width=640, height=480, out_path=None):
    # Setup renderer
    vis = o3d.visualization.rendering.OffscreenRenderer(width, height)
    scene = vis.scene
    scene.set_background([1, 1, 1, 1])  # white

    # Material
    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = "defaultLit"

    scene.add_geometry("mesh", mesh, material)

    # Setup camera
    cam = vis.scene.camera
    cam.setup_camera(intrinsic, extrinsic)

    # Render
    img = vis.render_to_image()

    if out_path:
        o3d.io.write_image(out_path, img)

    return img

def render(args, cam_poses, visualize=False):
    "Render with bullet renderer - deprecated"
    # Fixed up vector and projection matrix
    camera_up = [0, 0, 1]
    fov = 50
    aspect = 1.0
    near = 0.01
    far = 10
    width, height = 256, 256
    proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    # Render images
    images = []
    for cam_eye, cam_target in cam_poses:
        view_matrix = p.computeViewMatrix(cameraEyePosition=cam_eye,
                                          cameraTargetPosition=cam_target,
                                          cameraUpVector=camera_up)

        img = p.getCameraImage(width=width,
                                height=height,
                                viewMatrix=view_matrix,
                                projectionMatrix=proj_matrix,
                                lightDirection=[1, 1, 1],       # Bright directional light
                                lightColor=[1, 1, 1],           # White light
                                renderer=p.ER_TINY_RENDERER)

        rgb_array = np.reshape(img[2], (height, width, 4))[:, :, :3]  # Drop alpha
        images.append(rgb_array)

    # get the red pixels
    vis = np.concatenate(images, axis=1)
    red_blue = vis[..., 0] - vis[..., -1]
    score = red_blue.sum()
    # plt.imshow(red_blue)
    # plt.show()
    # Visualize
    # vis = np.concatenate(images, axis=1)
    if visualize:
        plt.imshow(vis)
        plt.show()
    return score

def get_action_space(args, meshes):
    """obtain the action space. action has format [object id, delta x, delta y]"""
    r_min=0
    r_max=0.05
    r_cnt=10
    actions = []
    for body_idx in meshes:
        # for each leaf node cluster, sample from action space
        for r in np.arange(r_min, r_max, (r_max - r_min) / r_cnt):
            for theta in np.arange(0, 2 * np.pi, 2 * np.pi / 8):
                actions.append([body_idx, r * np.cos(theta), r * np.sin(theta)])
        # for each body_idx, put to original pose
        actions.append([body_idx, 0, 0])

    return actions

def check_constraint(action, meshes, delta, dist_thresh=0.01):
    """Function to check the constraints"""
    # collision
    collision = False
    other_meshes = meshes.copy()
    other_meshes.remove(action[0])
    for other_mesh in other_meshes:
        dist = p.getClosestPoints(action[0], other_mesh, distance=10)
        if dist[0][8] < dist_thresh:
            collision = True
            return collision
    return collision

def load_py_config(file_path):
    spec = importlib.util.spec_from_file_location("config", file_path)
    config = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(config)
    return config

def get_intrinsic(args):
    # load the datapath
    config_path = os.path.join(os.path.dirname(args.path), "config.py")
    config = load_py_config(config_path)
    datadir = config.data['datadir']
    json_file = os.path.join(datadir, "transforms_train.json")
    with open(json_file, "r") as f:
        data = json.load(f)
    # Access individual values
    width = data["w"]
    height = data["h"]
    fx = data["fl_x"]
    fy = data["fl_y"]
    cx = data["cx"]
    cy = data["cy"]
    intrinsic = o3d.camera.PinholeCameraIntrinsic(int(width), int(height), fx, fy, cx, cy)
    return intrinsic

def load_mesh_with_mtl(obj_path):
    # Ensure textures can be found
    mesh = o3d.io.read_triangle_mesh(obj_path, enable_post_processing=True)
    if not mesh.has_vertex_normals():
        mesh.compute_vertex_normals()
    return mesh

def apply_translation(mesh, translation):
    mesh.translate(translation)
    return mesh

def render_view(mesh, intrinsic, extrinsic, width=640, height=480, out_path=None):
    # Setup renderer
    vis = o3d.visualization.rendering.OffscreenRenderer(width, height)
    scene = vis.scene
    scene.set_background([1, 1, 1, 1])  # white

    # Material
    material = o3d.visualization.rendering.MaterialRecord()
    material.shader = "defaultLit"

    scene.add_geometry("mesh", mesh, material)

    # Setup camera
    cam = vis.scene.camera
    cam.setup_camera(intrinsic, extrinsic)

    # Render
    img = vis.render_to_image()

    if out_path:
        o3d.io.write_image(out_path, img)

    return img

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("path", help='path to cluster dir')
    parser.add_argument("--cluster_idx", type=int, default=0, help='the index of the mesh to move')
    parser.add_argument("--max_dist", type=float, default=0.15, help='maximum deviation of the mesh (x, y)')
    args = parser.parse_args()
    max_dist = args.max_dist

    # load scene meshes and set up scene
    # p.connect(p.DIRECT)
    p.connect(p.GUI)
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
    # conf = (0, -np.pi / 4.0, 0, -3.0 * np.pi / 4.0, 0, np.pi / 2, np.pi / 4, 0.04, 0.04)
    # # move robot to start position
    # joints = get_movable_joints(robot)
    # set_joint_positions(robot, joints, conf)
    saved_world = WorldSaver()

    # Load leaf nodes and uncertainty-colored meshes
    leaf_node_idxs, mesh_paths = load_tree(args.path)
    # load the meshes into pybullet simulator
    meshes = []
    for mesh_path in mesh_paths:
        # if "mesh_03" in mesh_path:
        #     mesh = p.loadURDF(mesh_path)
        #     meshes.append(mesh)
        mesh = p.loadURDF(mesh_path)
        meshes.append(mesh)

    # let's set the camera poses
    cam_poses, center, z_min = set_camera_poses(args, meshes)

    # populate the scene with simple floor
    floor = p.loadURDF('models/table/table.urdf', globalScaling=1.0)
    floor_aabb_min, floor_aabb_max = p.getAABB(floor)
    height = floor_aabb_max[-1] - floor_aabb_min[-1]
    _, current_orientation = p.getBasePositionAndOrientation(floor)
    p.resetBasePositionAndOrientation(floor, [center[0], center[1], z_min - floor_aabb_max[-1]], current_orientation)

    # create the open3d rendering scene
    intrinsic = get_intrinsic(args)
    # load the meshes
    o3d_meshes = []
    for mesh_path in mesh_paths:
        mesh = load_mesh_with_mtl(mesh_path.replace('.urdf', ".obj"))
        o3d_meshes.append(mesh)
    o3d.visualization.draw_geometries(o3d_meshes)
    breakpoint()


    # let's sample the actions
    actions = get_action_space(args, meshes)
    scores = np.zeros(len(actions))
    # loop for all actions:
    for idx, action in enumerate(tqdm(actions)):
        # move the object
        delta = [action[1], action[2], 0]
        p.resetBasePositionAndOrientation(action[0], delta, [0, 0, 0, 1])
        # get the constraints - collision etc...
        collision = check_constraint(action, meshes, delta)
        # let's render and calculate the score
        if not collision:
            score = render(args, cam_poses)
        else:
            score = -1
        scores[idx] = score
    # debug - render max score action
    max_score_action = np.argmax(scores)
    delta = [actions[max_score_action][1], actions[max_score_action][2], 0]
    p.resetBasePositionAndOrientation(actions[max_score_action][0], delta, [0, 0, 0, 1])
    # let's render and calculate the score
    score = render(args, cam_poses, visualize=True)

    
    
    
    
    """ 
    # get the upper most layer clusters
    upper_nodes = np.loadtxt(os.path.join(args.path, "upper_nodes.txt")).astype(np.uint8).tolist()
    for i in upper_nodes:
        fname = os.path.join(args.path, "mesh_{:02d}.urdf".format(i))
        breakpoint()
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


    """ 