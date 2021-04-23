from __future__ import print_function

import argparse
import time
import random
import numpy as np
from collections import OrderedDict, defaultdict

from pybullet_tools.utils import HUSKY_URDF, load_model, TURTLEBOT_URDF, joints_from_names, \
    set_joint_positions, HideOutput, get_bodies, sample_placement, pairwise_collision, \
    add_data_path, load_pybullet, set_point, Point, create_box, stable_z, BLUE, TAN, GREY, connect, PI, \
    wait_if_gui, dump_body, set_all_color, BLUE, child_link_from_joint, link_from_name, draw_pose, Pose, pose_from_pose2d, \
    get_random_seed, get_numpy_seed, set_random_seed, set_numpy_seed

# TODO: https://stackoverflow.com/questions/1653970/does-python-have-an-ordered-set

BASE_LINK = 'base_link'
BASE_JOINTS = ['x', 'y', 'theta']

##################################################

def sample_placements(body_surfaces, obstacles=None, min_distances={}):
    # TODO: check object at the goal pose
    if obstacles is None:
        # TODO: savers per object
        obstacles = [body for body in get_bodies() if body not in body_surfaces]
    obstacles = list(obstacles)
    if not isinstance(min_distances, dict):
        min_distances = defaultdict(lambda: 0.)
    # TODO: max attempts here
    for body, surface in body_surfaces.items(): # TODO: shuffle
        min_distance = min_distances.get(body, 0.)
        while True:
            pose = sample_placement(body, surface)
            if pose is None:
                return False
            if not any(pairwise_collision(body, obst, max_distance=min_distance)
                       for obst in obstacles if obst not in [body, surface]):
                obstacles.append(body)
                break
    return True

##################################################

def problem1(n_obstacles=10, wall_side=0.1, obst_width=0.25, obst_height=0.5):
    floor_extent = 5.0
    base_limits = (-floor_extent/2.*np.ones(2),
                   +floor_extent/2.*np.ones(2))

    floor_height = 0.001
    floor = create_box(floor_extent, floor_extent, floor_height, color=TAN)
    set_point(floor, Point(z=-floor_height/2.))

    wall1 = create_box(floor_extent + wall_side, wall_side, wall_side, color=GREY)
    set_point(wall1, Point(y=floor_extent/2., z=wall_side/2.))
    wall2 = create_box(floor_extent + wall_side, wall_side, wall_side, color=GREY)
    set_point(wall2, Point(y=-floor_extent/2., z=wall_side/2.))
    wall3 = create_box(wall_side, floor_extent + wall_side, wall_side, color=GREY)
    set_point(wall3, Point(x=floor_extent/2., z=wall_side/2.))
    wall4 = create_box(wall_side, floor_extent + wall_side, wall_side, color=GREY)
    set_point(wall4, Point(x=-floor_extent/2., z=wall_side/2.))
    walls = [wall1, wall2, wall3, wall4]

    initial_surfaces = OrderedDict()
    for _ in range(n_obstacles):
        body = create_box(obst_width, obst_width, obst_height, color=GREY)
        initial_surfaces[body] = floor

    initial_conf = np.array([+floor_extent/3, -floor_extent/3, 3*PI/4])
    draw_pose(pose_from_pose2d(initial_conf), length=0.5)
    goal_conf = -initial_conf
    draw_pose(pose_from_pose2d(goal_conf), length=0.5)

    with HideOutput():
        robot = load_model(TURTLEBOT_URDF)
        base_joints = joints_from_names(robot, BASE_JOINTS)
        # base_link = child_link_from_joint(base_joints[-1])
        base_link = link_from_name(robot, BASE_LINK)
        set_all_color(robot, BLUE)
    dump_body(robot)
    set_point(robot, Point(z=stable_z(robot, floor)))
    #handles = draw_aabb(get_aabb(robot))
    #print(get_center_extent(robot))
    #wait_for_user()
    draw_pose(Pose(), parent=robot, parent_link=base_link, length=0.5)
    set_joint_positions(robot, base_joints, initial_conf)

    sample_placements(initial_surfaces, obstacles=[robot] + walls, min_distances=10e-2)

def main():
    parser = argparse.ArgumentParser()
    # parser.add_argument('-c', '--cfree', action='store_true',
    #                     help='When enabled, disables collision checking (for debugging).')
    # parser.add_argument('-p', '--problem', default='test_pour', choices=sorted(problem_fn_from_name),
    #                     help='The name of the problem to solve.')
    parser.add_argument('-s', '--seed', default=None, type=int,
                        help='The random seed to use.')
    parser.add_argument('-v', '--viewer', action='store_true',
                        help='')
    args = parser.parse_args()

    connect()

    seed = args.seed
    #seed = 0
    #seed = time.time()
    set_random_seed(seed=seed) # None: 2147483648 = 2**31
    set_numpy_seed(seed=seed)
    print('Random seed:', get_random_seed(), random.random())
    print('Numpy seed:', get_numpy_seed(), np.random.random())

    problem1()
    wait_if_gui()

if __name__ == '__main__':
    main()
