#!/usr/bin/env python3
"""Code to load the scene, grasp a object based on mesh. Need candidate grasps to be pre-calculated"""

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

def plan(robot, block, fixed, teleport):
    grasp_gen = get_grasp_gen(robot, 'top')
    ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport, num_attempts=100)
    free_motion_fn = get_free_motion_gen(robot, fixed=([block] + fixed), teleport=teleport)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=fixed, teleport=teleport)

    pose0 = BodyPose(block)
    conf0 = BodyConf(robot)
    saved_world = WorldSaver()
    for grasp, in grasp_gen(block):
        saved_world.restore()
        result1 = ik_fn(block, pose0, grasp)
        if result1 is None:
            continue
        conf1, path2 = result1
        pose0.assign()
        result2 = free_motion_fn(conf0, conf1)
        if result2 is None:
            continue
        path1, = result2
        result3 = holding_motion_fn(conf1, conf0, block, grasp)
        if result3 is None:
            continue
        path3, = result3
        # path1.body_paths[0].path[0] -> joint angles
        return Command(path1.body_paths +
                          path2.body_paths +
                          path3.body_paths)
    return None

def plan_in_motion(robot, target_points, target_width, target_mesh, scene_mesh, floor, teleport, max_grasp_width, smoothing=False, algorithm=None):
    # robot gripper width
    grasp_gen = get_grasp_gen(robot, 'top')
    # grasping_fn = get_grasping_fn(robot, fixed=scene_mesh, teleport=teleport, num_attempts=10)
    grasping_fn = get_grasping_fn(robot, fixed=[], teleport=teleport, num_attempts=10)
    free_motion_fn = get_free_motion_gen(robot, fixed=[], teleport=teleport, smoothing=smoothing, algorithm=algorithm)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=[], teleport=teleport, algorithm=algorithm)

    target_poses = []
    for target in target_points:
        target_poses.append(BodyPose(target))

    init_pos = BodyConf(robot)

    saved_world = WorldSaver()

    for i, (target_pose, target_point) in enumerate(zip(target_poses, target_points)):
        if target_width[i] > max_grasp_width:
            continue
        for grasp, in grasp_gen(target_point):
            saved_world.restore()
            # width = target_width[i] + 0.01
            # width = np.clip(width, a_min=0.005, a_max=0.04)
            # conf = (0, -np.pi / 4.0, 0, -3.0 * np.pi / 4.0, 0, np.pi / 2, np.pi / 4, width, width)
            # joints = get_movable_joints(robot)
            # set_joint_positions(robot, joints, conf)
    
            q_grasp = inverse_kinematics(robot, grasp.link,
                end_effector_from_body(target_pose.pose, grasp.grasp_pose))
            conf = BodyConf(robot, q_grasp)
            link_pose = get_link_pose(robot, link_from_name(robot, 'panda_hand'))

            in_motion_result = free_motion_fn(init_pos, conf) # move from initial position to grasp position
            if in_motion_result is None:
                continue
            in_motion_cmd, = in_motion_result
            in_motion_cmd = in_motion_cmd.refine(num_steps=50)
            return in_motion_cmd, target_pose

    return None, None


def plan_with_delta(robot, delta_pos, target_ori, other_cluster_convexes=[], algorithm='direct'):
    move_target = create_box(w=0.01, l=0.01, h=0.01, color=RGBA(0, 0, 1, 1))
    grasp_gen = get_grasp_gen(robot, 'top')
    free_motion_fn = get_free_motion_gen(robot, fixed=[], teleport=False, smoothing=False, algorithm=algorithm)
    current_pos = get_link_pose(robot, link_from_name(robot, 'tool_link'))[0]
    target_point = [current_pos[0] + delta_pos[0],
                    current_pos[1]+ delta_pos[1],
                    current_pos[2]+ delta_pos[2]
                    ]
    p.resetBasePositionAndOrientation(move_target, target_point, target_ori)
    target_pose = BodyPose(move_target)
    init_pos = BodyConf(robot)

    saved_world = WorldSaver()
    for grasp, in grasp_gen(move_target):
        saved_world.restore()
        q_grasp = inverse_kinematics(robot, grasp.link,
            end_effector_from_body(target_pose.pose, grasp.grasp_pose))
        conf = BodyConf(robot, q_grasp)
        link_pose = get_link_pose(robot, link_from_name(robot, 'panda_hand'))
        motion_result = free_motion_fn(init_pos, conf) # move from initial position to grasp position
        if motion_result is None:
            continue
        motion_cmd, = motion_result
        motion_cmd = motion_cmd.refine(num_steps=50)
        return motion_cmd

    return None
   

def plan_nudge_in(robot, in_points, out_points, target_width, target_mesh, scene_mesh, floor, teleport, smoothing=False, algorithm=None):
    # close the gripper
    conf = (0, -np.pi / 4.0, 0, -3.0 * np.pi / 4.0, 0, np.pi / 2, np.pi / 4, 0.003, 0.003)
    joints = get_movable_joints(robot)
    set_joint_positions(robot, joints, conf)
 
    # robot gripper width
    grasp_gen = get_grasp_gen(robot, 'nudge')
    grasping_fn = get_grasping_fn(robot, fixed=scene_mesh, teleport=teleport, num_attempts=10)
    free_motion_fn = get_free_motion_gen(robot, fixed=scene_mesh, teleport=teleport, smoothing=smoothing, algorithm=algorithm)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=scene_mesh, teleport=teleport, algorithm=algorithm)

    in_poses = []
    for in_point in in_points:
        in_poses.append(BodyPose(in_point))

    init_pos = BodyConf(robot)

    saved_world = WorldSaver()

    for i, (in_pose, in_point) in enumerate(zip(in_poses, in_points)):
       for grasp, in grasp_gen(in_point):
            saved_world.restore()
            q_grasp = inverse_kinematics(robot, grasp.link,
                end_effector_from_body(in_pose.pose, grasp.grasp_pose))
            conf = BodyConf(robot, q_grasp)
            in_motion_result = free_motion_fn(init_pos, conf) # move from initial position to grasp position
            if in_motion_result is not None:
                in_motion_cmd, = in_motion_result
                in_motion_cmd = in_motion_cmd.refine(num_steps=50)
                return in_motion_cmd, conf, i
    return None


def plan_nudge_out(robot, out_point, conf, other_cluster_convexes, floor, teleport, smoothing=False, algorithm=None):
    # go to the end conf of nudge in
    # joints = get_movable_joints(robot)
    # set_joint_positions(robot, joints, conf)
 
    grasp_gen = get_grasp_gen(robot, 'nudge')
    grasping_fn = get_grasping_fn(robot, fixed=other_cluster_convexes, teleport=teleport, num_attempts=10)
    free_motion_fn = get_free_motion_gen(robot, fixed=other_cluster_convexes, teleport=teleport, smoothing=smoothing, algorithm=algorithm)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=other_cluster_convexes, teleport=teleport, algorithm=algorithm)

    out_pose = BodyPose(out_point)
    init_pos = BodyConf(robot)

    saved_world = WorldSaver()

    for grasp, in grasp_gen(out_point):
        saved_world.restore()
        q_grasp = inverse_kinematics(robot, grasp.link,
            end_effector_from_body(out_pose.pose, grasp.grasp_pose))
        conf = BodyConf(robot, q_grasp)
        out_motion_result = free_motion_fn(init_pos, conf) # move from initial position to grasp position
        if out_motion_result is not None:
            out_motion_cmd, = out_motion_result
            out_motion_cmd = out_motion_cmd.refine(num_steps=50)
            return out_motion_cmd, out_pose
    return None


def perpendicular_euler_angles(euler_angles):
    """
    Generates a set of Euler angles that represents a rotation perpendicular to the input rotation.
    """
    # Convert the input Euler angles to a rotation matrix
    r = R.from_euler('xyz', euler_angles, degrees=True).as_matrix()

    # Compute the direction perpendicular to the input z-axis
    z_dir = np.array([0, 0, 1])
    perp_dir = np.cross(z_dir, r[:,2])

    # Generate a random angle of rotation
    angle = np.random.uniform(0, 2*np.pi)

    # Rotate the perpendicular direction about the input z-axis by the random angle
    rot_axis = r[:,2]
    rot = R.from_rotvec(angle * rot_axis)
    perp_dir_rotated = rot.apply(perp_dir)

    # Convert the rotated direction to Euler angles
    yaw = np.arctan2(perp_dir_rotated[1], perp_dir_rotated[0])
    pitch = np.arctan2(np.sqrt(perp_dir_rotated[0]**2 + perp_dir_rotated[1]**2), perp_dir_rotated[2])
    roll = 0.0

    # Convert the new Euler angles to degrees and return them
    return np.array([roll, pitch, yaw]) * 180 / np.pi

def get_orthogonal_vector(direction):
    """
    Finds an orthogonal vector to the given directional vector using the cross product.
    """
    # Choose a random vector to cross with the input vector
    rand_vec = np.random.randn(3)

    # Take the cross product of the input vector and the random vector
    ortho_vec = np.cross(direction, rand_vec)

    # If the cross product is zero, choose a different random vector
    while np.linalg.norm(ortho_vec) < 1e-6:
        rand_vec = np.random.randn(3)
        ortho_vec = np.cross(direction, rand_vec)

    # Normalize the orthogonal vector and return it
    return ortho_vec / np.linalg.norm(ortho_vec)

def vector_to_euler(direction):
    """
    Converts a directional vector to Euler angles.
    """
    yaw = np.arctan2(direction[1], direction[0])
    pitch = np.arctan2(direction[2], np.sqrt(direction[0]**2 + direction[1]**2))
    roll = 0.0  # There is no unique solution for roll

    return np.array([yaw, pitch, roll]).tolist()

def vector_to_roll(direction):
    """Function to change directional vector to roll"""
    yaw = 0.0
    pitch = 0.0
    roll = np.arctan2(direction[1], -direction[0])  # There is no unique solution for roll

    return np.array([yaw, pitch, roll]).tolist()


def main(path, cluster_cnt, panda_ip, send_to_panda, z_offset, max_grasp_width, sample=2, display='execute'): # control | execute | step
    # read the inputs
    # the grasp data - [x1, y1, z1, x2, y2, z2]
    grasp_data = np.loadtxt(os.path.join(path, "mesh_{:02d}.txt".format(cluster_cnt)))
    grasp_data = grasp_data[:sample]
    x1_arr = grasp_data[:, :3]
    x2_arr = grasp_data[:, 3:]
    # get the avg height of the first grasp (the highest)
    z_height = (x1_arr[0, -1] + x2_arr[0, -1]) * 0.5
    # all_urdf_names = sorted(glob(os.path.join(path, "mesh_*.urdf")))
    center_arr = ((x1_arr + x2_arr) / 2).tolist()
    direction_arr = x1_arr - x2_arr
    direction_arr = direction_arr / np.linalg.norm(direction_arr)
    target_width = np.linalg.norm(x1_arr - x2_arr, axis=1)


    connect(use_gui=True)
    disable_real_time()
    # add_data_path()
    draw_global_system()
    with HideOutput():
        # robot = load_model(FRANKA_URDF) # KUKA_IIWA_URDF | DRAKE_IIWA_URDF
        robot = load_pybullet(FRANKA_URDF, fixed_base=True)
        # robot = load_pybullet(FRANKA_URDF.replace('panda_arm_hand', 'panda_arm_hand_cam_notip'), fixed_base=True)
        # floor = p.loadURDF('plane.urdf')
        floor = load_model('models/short_floor.urdf')
        set_pose(floor, Pose(Point(x=1.2,  z=-0.015 + z_offset)))
    set_camera_pose(camera_point=[0.8, -0.2, 0.8])
    conf = (0, -np.pi / 4.0, 0, -3.0 * np.pi / 4.0, 0, np.pi / 2, np.pi / 4, 0.04, 0.04)
    # # move robot to start position
    joints = get_movable_joints(robot)
    set_joint_positions(robot, joints, conf)
    saved_world = WorldSaver()

    # let's keep track of other cluster's convexes
    other_cluster_convexes = []
    # Load the scene broken down in convex
    scene_mesh = []
    # for fname in sorted(glob(os.path.join(path, 'convex/convex_*_part_*.urdf'))):
    for fname in sorted(glob(os.path.join(path, 'mesh_*.urdf'))):
        mesh = p.loadURDF(fname, basePosition=(0, 0, z_offset))
        scene_mesh.append(mesh)
        # convex_idx = fname.split('/')[-1]
        # convex_idx = int(convex_idx.split('_')[1])
        # if convex_idx != cluster_cnt:
        #     other_cluster_convexes.append(mesh)
 
    ########################################### motion for grasping ###############################################
    if z_height > args.z_thresh:
        perp_targets = []
        for _ in range(len(x1_arr)):
            # grasp_target1 = create_box(w=0.01, l=0.01, h=0.01, color=RGBA(0, 1, 0, 1)) # Creates a red box obstacle
            # grasp_target2 = create_box(w=0.01, l=0.01, h=0.01, color=RGBA(0, 1, 0, 1)) # Creates a red box obstacle
            perp_target = create_box(w=0.01, l=0.01, h=0.01, color=RGBA(1, 0, 0, 1)) # Creates a red box obstacle
            perp_targets.append(perp_target)

            # set_point(obstacle, [0.5, 0.5, 0.1 / 2.]) # Sets the [x,y,z] position of the obstacle
            # print('Position:', get_point(obstacle))
            # set_euler(obstacle, [0, 0, np.pi / 4]) #  Sets the [roll,pitch,yaw] orientation of the obstacle
            # print('Orientation:', get_euler(obstacle))
        # set_pose(grasp_target, Pose(Point(x=0.39, y=0.03, z=0.15), [np.pi / 2, np.pi / 2, np.pi / 2]))

        for i, (perp_target, center, direction) in enumerate(zip(perp_targets, center_arr, direction_arr)):
            set_pose(perp_target, Pose(Point(x=center[0], y=center[1], z=center[2] + z_offset), vector_to_roll(direction)))
        
        # go down to grasp the robot
        in_motion_cmd, target_pose = plan_in_motion(robot,
                                        target_points=perp_targets,
                                        target_width=target_width,
                                        target_mesh=None,
                                        scene_mesh=scene_mesh,
                                        floor=floor,
                                        teleport=False,
                                        max_grasp_width=max_grasp_width,
                                        smoothing=False
                                        )
                                        # algorithm='direct')
        # the orientation of the reached target
        target_ori = target_pose.pose[1]
        # let's get the delta pos
        delta_pos_path = os.path.join(path, "delta_pos_{}.txt".format(cluster_cnt))
        if not os.path.exists(delta_pos_path):
            print("No delta pos for cluster {} found!".format(cluster_cnt))
            print("Please run python find_free_space <path> <cluster_cnt>")
        delta_pos = np.loadtxt(delta_pos_path)

        # move up and to the delta pos
        lift_motion = plan_with_delta(robot, delta_pos=[0, 0, 0.1], target_ori=target_ori, other_cluster_convexes=other_cluster_convexes)
        move_motion = plan_with_delta(robot, delta_pos=delta_pos, target_ori=target_ori, other_cluster_convexes=other_cluster_convexes)
        down_motion = plan_with_delta(robot, delta_pos=[0, 0, 0.05], target_ori=target_ori, other_cluster_convexes=other_cluster_convexes)
        lift_motion2 = plan_with_delta(robot, delta_pos=[0, 0, 0.2], target_ori=target_ori, other_cluster_convexes=other_cluster_convexes)

        command = Command(in_motion_cmd.body_paths + lift_motion.body_paths + move_motion.body_paths + down_motion.body_paths + lift_motion2.body_paths)

        in_motion_path = [pos for path in in_motion_cmd.body_paths for pos in path.path]
        in_motion_path = np.array(in_motion_path)
        np.savetxt(os.path.join(path, "in_motion_{}.txt".format(cluster_cnt)), in_motion_path)

        lift_motion_path = [pos for path in lift_motion.body_paths for pos in path.path]
        lift_motion_path = np.array(lift_motion_path)
        np.savetxt(os.path.join(path, "lift_motion_{}.txt".format(cluster_cnt)), lift_motion_path)
        
        move_motion_path = [pos for path in move_motion.body_paths for pos in path.path]
        move_motion_path = np.array(move_motion_path)
        np.savetxt(os.path.join(path, "move_motion_{}.txt".format(cluster_cnt)), move_motion_path)

        down_motion_path = [pos for path in down_motion.body_paths for pos in path.path]
        down_motion_path = np.array(down_motion_path)
        np.savetxt(os.path.join(path, "down_motion_{}.txt".format(cluster_cnt)), down_motion_path)
        
        lift_motion2_path = [pos for path in lift_motion2.body_paths for pos in path.path]
        lift_motion2_path = np.array(lift_motion2_path)
        np.savetxt(os.path.join(path, "lift_motion2_{}.txt".format(cluster_cnt)), lift_motion2_path)



        # let's send to panda if required
        if send_to_panda:
            panda_path = "panda@{}:/home/panda/Downloads/grasp_trajs".format(panda_ip)
            cmd = "scp -P 7910 {} {}".format(os.path.join(path, "in_motion_{}.txt".format(cluster_cnt)), panda_path)
            os.system(cmd)
            cmd = "scp -P 7910 {} {}".format(os.path.join(path, "lift_motion_{}.txt".format(cluster_cnt)), panda_path)
            os.system(cmd)
            cmd = "scp -P 7910 {} {}".format(os.path.join(path, "move_motion_{}.txt".format(cluster_cnt)), panda_path)
            os.system(cmd)
            cmd = "scp -P 7910 {} {}".format(os.path.join(path, "down_motion_{}.txt".format(cluster_cnt)), panda_path)
            os.system(cmd)
            cmd = "scp -P 7910 {} {}".format(os.path.join(path, "lift_motion2_{}.txt".format(cluster_cnt)), panda_path)
            os.system(cmd)

        if (command is None) or (display is None):
            print('Unable to find a plan!')
            return
    
    ########################################### motion for nudging ###############################################
    else:
        # let's get the center point
        center = np.loadtxt(os.path.join(path, "mesh_{:02d}_center.txt".format(cluster_cnt)))
        x_arr = np.concatenate((x1_arr, x2_arr), axis=0)
        in_targets = []
        out_targets = []
        for _ in range(len(x_arr)):
            grasp_target1 = create_box(w=0.01, l=0.01, h=0.01, color=RGBA(1, 0, 0, 1)) # Creates a red box obstacle
            grasp_target2 = create_box(w=0.01, l=0.01, h=0.01, color=RGBA(0, 1, 0, 1)) # Creates a green box obstacle
            in_targets.append(grasp_target2)
            out_targets.append(grasp_target1)

        for i, (in_target, out_target, x1_pos, direction) in enumerate(zip(in_targets, out_targets, x_arr, direction_arr)):
            set_pose(in_target, Pose(Point(x=x1_pos[0], y=x1_pos[1], z=x1_pos[2] + z_offset), vector_to_roll(direction)))
            set_pose(out_target, Pose(Point(x=2 * center[0] - x1_pos[0], y=2 * center[1] - x1_pos[1], z=2 * center[2] - x1_pos[2]) + z_offset, vector_to_roll(direction)))
        
        # get the motion to approach
        in_motion_cmd, conf, target_idx = plan_nudge_in(robot,
                                        in_points=in_targets,
                                        out_points=out_targets,
                                        target_width=target_width,
                                        target_mesh=None,
                                        scene_mesh=scene_mesh,
                                        floor=floor,
                                        teleport=False,
                                        smoothing=False,
                                        algorithm='direct')
        if in_motion_cmd is None:
            print("===========================Could not go to nudge starting point=========================")
            os._exit(0)
        out_motion_cmd, out_pose = plan_nudge_out(robot,
                                        out_point=out_targets[target_idx],
                                        conf=conf,
                                        other_cluster_convexes=other_cluster_convexes,
                                        floor=floor,
                                        teleport=False,
                                        smoothing=False,
                                        algorithm='direct'

        )
        lift_motion = plan_with_delta(robot, delta_pos=[0, 0, 0.2], target_ori=out_pose.pose[-1], other_cluster_convexes=other_cluster_convexes)
        command = Command(in_motion_cmd.body_paths + out_motion_cmd.body_paths + lift_motion.body_paths)
        in_motion_path = [pos for path in in_motion_cmd.body_paths for pos in path.path]
        in_motion_path = np.array(in_motion_path)
        np.savetxt(os.path.join(path, "in_motion_{}.txt".format(cluster_cnt)), in_motion_path)
        
        out_motion_path = [pos for path in out_motion_cmd.body_paths for pos in path.path]
        out_motion_path = np.array(out_motion_path)
        np.savetxt(os.path.join(path, "nudge_motion_{}.txt".format(cluster_cnt)), out_motion_path)

        lift_motion_path = [pos for path in lift_motion.body_paths for pos in path.path]
        lift_motion_path = np.array(lift_motion_path)
        np.savetxt(os.path.join(path, "lift_motion_{}.txt".format(cluster_cnt)), lift_motion_path)


        # let's send to panda if required
        if send_to_panda:
            panda_path = "panda@{}:/home/panda/Downloads/grasp_trajs".format(panda_ip)
            cmd = "scp -P 7910 {} {}".format(os.path.join(path, "in_motion_{}.txt".format(cluster_cnt)), panda_path)
            os.system(cmd)
            cmd = "scp -P 7910 {} {}".format(os.path.join(path, "nudge_motion_{}.txt".format(cluster_cnt)), panda_path)
            os.system(cmd)
            cmd = "scp -P 7910 {} {}".format(os.path.join(path, "lift_motion_{}.txt".format(cluster_cnt)), panda_path)
            os.system(cmd)
 
        if (command is None) or (display is None):
            print('Unable to find a plan!')
            return
 
    saved_world.restore()
    update_state()
    breakpoint()
    if display == 'control':
        enable_gravity()
        command.control(real_time=False, dt=0)
    elif display == 'execute':
        command.execute(time_step=0.002)
    elif display == 'step':
        command.step()
    else:
        raise ValueError(display)

    if (command is None) or (display is None):
        print('Unable to find a plan!')
        return

    print('Quit?')
    wait_if_gui()
    disconnect()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("path", help='path to cluster dir')
    parser.add_argument("--cluster_idx", type=int, default=0)
    parser.add_argument("--panda_ip", type=str, default='147.46.132.81')
    parser.add_argument("--send_to_panda", action='store_true')
    parser.add_argument("--z_thresh", type=float, default=0.05, help='nudge objects under this height')
    parser.add_argument("--z_offset", type=float, default=0.04, help='lift all ground and objects')
    parser.add_argument("--max_grasp_width", type=float, default=0.04, help='max grasp width')
    parser.add_argument("--n_sample", type=int, default=50, help='number of potential grasps to consider')
    args = parser.parse_args()
    main(args.path,
         args.cluster_idx, 
         args.panda_ip, 
         args.send_to_panda,
         args.z_offset,
         args.max_grasp_width,
         args.n_sample,
         'execute')
