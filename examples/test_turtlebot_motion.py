from __future__ import print_function

import random
import numpy as np
from collections import OrderedDict

from pybullet_tools.utils import HUSKY_URDF, load_model, TURTLEBOT_URDF, joints_from_names, \
    set_joint_positions, HideOutput, get_bodies, sample_placement, pairwise_collision, \
    add_data_path, load_pybullet, set_point, Point, create_box, stable_z, BLUE, TAN, GREY, connect, PI, wait_if_gui

BASE_JOINTS = ['x', 'y', 'theta']

def sample_placements(body_surfaces, obstacles=None, min_distances={}):
    if obstacles is None:
        obstacles = [body for body in get_bodies() if body not in body_surfaces]
    obstacles = list(obstacles)
    # TODO: max attempts here
    for body, surface in body_surfaces.items():
        min_distance = min_distances.get(body, 0.01)
        while True:
            pose = sample_placement(body, surface)
            if pose is None:
                return False
            if not any(pairwise_collision(body, obst, max_distance=min_distance)
                       for obst in obstacles if obst not in [body, surface]):
                obstacles.append(body)
                break
    return True

def problem1(n_rovers=2, n_objectives=4, n_rocks=3, n_soil=3, n_stores=1, n_obstacles=8):
    base_extent = 5.0
    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))
    mount_width = 0.5
    mound_height = 0.1

    floor = create_box(base_extent, base_extent, 0.001, color=TAN) # TODO: two rooms
    set_point(floor, Point(z=-0.001/2.))

    wall1 = create_box(base_extent + mound_height, mound_height, mound_height, color=GREY)
    set_point(wall1, Point(y=base_extent/2., z=mound_height/2.))
    wall2 = create_box(base_extent + mound_height, mound_height, mound_height, color=GREY)
    set_point(wall2, Point(y=-base_extent/2., z=mound_height/2.))
    wall3 = create_box(mound_height, base_extent + mound_height, mound_height, color=GREY)
    set_point(wall3, Point(x=base_extent/2., z=mound_height/2.))
    wall4 = create_box(mound_height, base_extent + mound_height, mound_height, color=GREY)
    set_point(wall4, Point(x=-base_extent/2., z=mound_height/2.))
    # TODO: can add obstacles along the wall

    wall = create_box(mound_height, base_extent, mound_height, color=GREY)  # TODO: two rooms
    set_point(wall, Point(z=mound_height / 2.))

    add_data_path()
    with HideOutput():
        lander = load_pybullet(HUSKY_URDF, scale=1)
    lander_z = stable_z(lander, floor)
    set_point(lander, Point(-1.9, -2, lander_z))

    mound1 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound1, [+2, 2, mound_height/2.])
    mound2 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound2, [-2, 2, mound_height/2.])
    mound3 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound3, [+0.5, 2, mound_height/2.])
    mound4 = create_box(mount_width, mount_width, mound_height, color=GREY)
    set_point(mound4, [-0.5, 2, mound_height/2.])
    mounds = [mound1, mound2, mound3, mound4]
    random.shuffle(mounds)

    body_types = []
    initial_surfaces = OrderedDict()
    min_distances = {}
    for _ in range(n_obstacles):
        body = create_box(mound_height, mound_height, 4*mound_height, color=GREY)
        initial_surfaces[body] = floor

    # camera_rgb_optical_frame
    with HideOutput():
        rover = load_model(TURTLEBOT_URDF)
    robot_z = stable_z(rover, floor)

    set_point(rover, Point(z=robot_z))
    #handles = draw_aabb(get_aabb(rover)) # Includes the origin
    #print(get_center_extent(rover))
    #wait_for_user()
    base_joints = joints_from_names(rover, BASE_JOINTS)
    set_joint_positions(rover, base_joints, [+1, -1.75, PI])
    #dump_body(rover)
    #draw_pose(get_link_pose(rover, link_from_name(rover, KINECT_FRAME)))

    obj_width = 0.07
    obj_height = 0.2

    objectives = []
    for i in range(n_objectives):
        body = create_box(obj_width, obj_width, obj_height, color=BLUE)
        objectives.append(body)
        #initial_surfaces[body] = random.choice(mounds)
        initial_surfaces[body] = mounds[i]

    min_distances.update({r: 0.05 for r in objectives})
    sample_placements(initial_surfaces, min_distances=min_distances)

def main():
    connect(use_gui=True)
    problem1()
    wait_if_gui()

if __name__ == '__main__':
    main()
