from __future__ import print_function

import argparse
import random
import numpy as np
import time
import math
from collections import OrderedDict, defaultdict
from itertools import combinations

from pybullet_tools.utils import load_model, TURTLEBOT_URDF, joints_from_names, \
    set_joint_positions, HideOutput, get_bodies, sample_placement, pairwise_collision, \
    set_point, Point, create_box, stable_z, TAN, GREY, connect, PI, OrderedSet, \
    wait_if_gui, dump_body, set_all_color, BLUE, child_link_from_joint, link_from_name, draw_pose, Pose, pose_from_pose2d, \
    get_random_seed, get_numpy_seed, set_random_seed, set_numpy_seed, plan_joint_motion, plan_nonholonomic_motion, \
    joint_from_name, safe_zip, draw_base_limits, BodySaver, WorldSaver, LockRenderer, elapsed_time, disconnect, flatten, \
    INF, wait_for_duration, get_unbuffered_aabb, draw_aabb, DEFAULT_AABB_BUFFER, get_link_pose, get_joint_positions, \
    get_subtree_aabb, get_pairs, get_distance_fn, get_aabb, set_all_static, step_simulation, get_bodies_in_region, \
    AABB, Profiler, pairwise_link_collision, BASE_LINK, get_collision_data, draw_pose2d, \
    normalize_interval, wrap_angle, CIRCULAR_LIMITS, wrap_interval, Euler, rescale_interval, adjust_path, \
    contact_collision, timer, update_scene, set_aabb_buffer, set_separating_axis_collisions, get_aabb, set_pose, \
    Pose, get_all_links, can_collide, aabb_overlap, set_collision_pair_mask, randomize, DEFAULT_RESOLUTION, \
    base_aligned_z, load_pybullet, get_collision_fn, get_custom_limits, get_limits_fn, \
    get_joint_velocities, control_joint, get_time_step, remove_handles, Interval, get_distance, \
    get_duration_fn, velocity_control_joint, get_max_velocities, get_difference

from motion_planners.trajectory.smooth import smooth_curve
from motion_planners.trajectory.linear import solve_multi_linear, quickest_inf_accel
from motion_planners.trajectory.limits import check_spline
from motion_planners.utils import waypoints_from_path, default_selector, irange
from motion_planners.trajectory.discretize import time_discretize_curve

from pybullet_tools.retime import interpolate_path, sample_curve

BASE_LINK_NAME = 'base_link'
BASE_JOINTS = ['x', 'y', 'theta']
DRAW_Z = 1e-3
DRAW_LENGTH = 0.5
MIN_AABB_VOLUME = DEFAULT_AABB_BUFFER**3

MAX_VELOCITIES = np.array([1., 1., PI / 4])
MAX_ACCELERATIONS = MAX_VELOCITIES / 0.25

#MAX_VELOCITIES *= INF
#MAX_ACCELERATIONS *= INF

MIN_PROXIMITY = 1e-2
N_DIGITS = 5

##################################################

def create_custom_base_limits(robot, base_limits):
    return {joint_from_name(robot, joint): limits
            for joint, limits in safe_zip(BASE_JOINTS[:2], zip(*base_limits))}

def sample_placements(body_surfaces, obstacles=None, savers=[], min_distances={}):
    if obstacles is None:
        obstacles = set(get_bodies()) - set(body_surfaces)
    savers = list(savers) + [BodySaver(obstacle) for obstacle in obstacles]
    if not isinstance(min_distances, dict):
        min_distances = {body: min_distances for body in body_surfaces}
    # TODO: max attempts here
    for body, surface in body_surfaces.items(): # TODO: shuffle
        min_distance = min_distances.get(body, 0.)
        while True:
            pose = sample_placement(body, surface)
            if pose is None:
                for saver in savers:
                    saver.restore()
                return False
            for saver in savers:
                obstacle = saver.body
                if obstacle in [body, surface]:
                    continue
                saver.restore()
                if pairwise_collision(body, obstacle, max_distance=min_distance):
                    break
            else:
                savers.append(BodySaver(body))
                break
    for saver in savers:
        saver.restore()
    return True

##################################################

def draw_waypoint(conf, z=DRAW_Z):
    return draw_pose(pose_from_pose2d(conf, z=z), length=DRAW_LENGTH)

def draw_conf(pose2d, interval, base_z, **kwargs):
    return draw_pose2d(pose2d, z=base_z + rescale_interval(
        wrap_interval(pose2d[2], interval=interval), old_interval=interval, new_interval=(-0.5, 0.5)), **kwargs)

def draw_path(path2d, z=DRAW_Z, **kwargs):
    if path2d is None:
        return []
    #return list(flatten(draw_pose(pose_from_pose2d(pose2d, z=z), **kwargs) for pose2d in path2d))
    #return list(flatten(draw_pose2d(pose2d, z=z, **kwargs) for pose2d in path2d))
    base_z = 1.
    start = path2d[0]
    mid_yaw = start[2]
    #mid_yaw = wrap_interval(mid_yaw)
    interval = (mid_yaw - PI, mid_yaw + PI)
    #interval = CIRCULAR_LIMITS
    draw_pose(pose_from_pose2d(start, z=base_z), length=1, **kwargs)
    # TODO: draw the current pose
    # TODO: line between orientations when there is a jump
    return list(flatten(draw_conf(pose2d, interval, base_z, **kwargs) for pose2d in path2d))

def extract_full_path(robot, path_joints, path, all_joints):
    with BodySaver(robot):
        new_path = []
        for conf in path:
            set_joint_positions(robot, path_joints, conf)
            new_path.append(get_joint_positions(robot, all_joints)) # TODO: do without assigning
        return new_path

def plan_motion(robot, joints, goal_positions, attachments=[], obstacles=None, holonomic=False, reversible=False, **kwargs):
    attached_bodies = [attachment.child for attachment in attachments]
    moving_bodies = [robot] + attached_bodies
    if obstacles is None:
        obstacles = get_bodies()
    obstacles = set(obstacles) - set(moving_bodies)
    if holonomic:
        return plan_joint_motion(robot, joints, goal_positions,
                                 attachments=attachments, obstacles=obstacles, **kwargs)
    # TODO: just sample the x, y waypoint and use the resulting orientation
    # TODO: remove overlapping configurations/intervals due to circular joints
    return plan_nonholonomic_motion(robot, joints, goal_positions, reversible=reversible,
                                    linear_tol=1e-6, angular_tol=0.,
                                    attachments=attachments, obstacles=obstacles, **kwargs)

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
    pillars = list(initial_surfaces)
    obstacles = walls + pillars

    initial_conf = np.array([+floor_extent/3, -floor_extent/3, 3*PI/4])
    goal_conf = -initial_conf

    robot = load_pybullet(TURTLEBOT_URDF, fixed_base=True, merge=True, sat=False)
    base_joints = joints_from_names(robot, BASE_JOINTS)
    # base_link = child_link_from_joint(base_joints[-1])
    base_link = link_from_name(robot, BASE_LINK_NAME)
    set_all_color(robot, BLUE)
    dump_body(robot)
    set_point(robot, Point(z=stable_z(robot, floor)))
    #set_point(robot, Point(z=base_aligned_z(robot)))
    draw_pose(Pose(), parent=robot, parent_link=base_link, length=0.5)
    set_joint_positions(robot, base_joints, initial_conf)

    sample_placements(initial_surfaces, obstacles=[robot] + walls,
                      savers=[BodySaver(robot, joints=base_joints, positions=goal_conf)],
                      min_distances=10e-2)

    # The first calls appear to be the slowest
    # times = []
    # for body1, body2 in combinations(pillars, r=2):
    #     start_time = time.time()
    #     colliding = pairwise_collision(body1, body2)
    #     runtime = elapsed_time(start_time)
    #     print(colliding, runtime)
    #     times.append(runtime)
    # print(times)

    return robot, base_limits, goal_conf, obstacles

##################################################

def compute_cost(robot, joints, path, resolutions=None):
    if path is None:
        return INF
    distance_fn = get_distance_fn(robot, joints, weights=resolutions) # TODO: get_duration_fn
    return sum(distance_fn(*pair) for pair in get_pairs(path))

def get_curve_collision_fn(robot, joints, custom_limits={}, resolutions=None, v_max=None, a_max=None, **kwargs):
    collision_fn = get_collision_fn(robot, joints, custom_limits=custom_limits, **kwargs)
    limits_fn = get_limits_fn(robot, joints, custom_limits)

    def curve_collision_fn(curve, t0, t1):
        if curve is None:
            return True
        # TODO: can exactly compute limit violations
        # if not check_spline(curve, v_max=max_velocities, a_max=None, verbose=False,
        #                     #start_t=t0, end_t=t1,
        #                     ):
        #     return True
        _, samples = time_discretize_curve(curve, verbose=False,
                                           #start_t=t0, end_t=t1,
                                           resolution=resolutions,
                                           #max_velocities=v_max,
                                           )
        if any(map(limits_fn, samples)):
           return True
        if any(map(collision_fn, default_selector(samples))):
           return True
        return False
    return curve_collision_fn

##################################################

def mpc(x0, v0, curve, dt_max=0.5, max_time=INF, max_iterations=INF, v_max=None, **kwargs):
    assert (max_time < INF) or (max_iterations < INF)
    from scipy.interpolate import CubicHermiteSpline
    start_time = time.time()
    best_cost, best_spline = INF, None
    for iteration in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break
        t1 = random.uniform(curve.x[0], curve.x[-1])
        future = (curve.x[-1] - t1) # TODO: weighted
        if future >= best_cost:
            continue
        x1 = curve(t1)
        if (v_max is not None) and (max((x1 - x0) / v_max) > dt_max):
            continue
        # if quickest_inf_accel(x0, x1, v_max=v_max) > dt_max:
        #     continue
        v1 = curve(t1, nu=1)
        #dt = dt_max
        dt = random.uniform(0, dt_max)
        times = [0., dt]
        positions = [x0, x1]
        velocities = [v0, v1]
        spline = CubicHermiteSpline(times, positions, dydx=velocities)
        if not check_spline(spline, **kwargs):
            continue
        # TODO: optimize to find the closest on the path within a range

        cost = future + (spline.x[-1] - spline.x[0])
        if cost < best_cost:
            best_cost, best_spline = cost, spline
            print('Iteration: {} | Cost: {:.3f} | T: {:.3f} | Time: {:.3f}'.format(
                iteration, cost, t1, elapsed_time(start_time)))
            print(best_cost, t1, elapsed_time(start_time))
    return best_cost, best_spline

def find_closest(x0, curve, t_range=None, max_time=INF, max_iterations=INF, distance_fn=None, verbose=False):
    assert (max_time < INF) or (max_iterations < INF)
    if t_range is None:
        t_range = Interval(curve.x[0], curve.x[-1])
    t_range = Interval(max(t_range[0], curve.x[0]), min(curve.x[-1], t_range[-1]))
    if distance_fn is None:
        distance_fn = get_distance
    start_time = time.time()
    closest_dist, closest_t = INF, None
    for iteration in irange(max_iterations):
        if elapsed_time(start_time) >= max_time:
            break
        t = random.uniform(*t_range) # TODO: halton
        x = curve(t)
        dist = distance_fn(x0, x)
        if dist < closest_dist:
            closest_dist, closest_t = dist, t
            if verbose:
                print('Iteration: {} | Dist: {:.3f} | T: {:.3f} | Time: {:.3f}'.format(
                    iteration, closest_dist, t, elapsed_time(start_time)))
    return closest_dist, closest_t

##################################################

def max_velocity_control_joints(robot, joints, positions=None, velocities=None, max_velocities=None):
    if velocities is None:
        velocities = np.zeros(len(joints))
    if max_velocities is None:
        max_velocities = get_max_velocities(robot, joints)
    for idx, joint in enumerate(joints):
        if positions is not None:
            control_joint(robot, joint=joint, position=positions[idx],
                          # velocity=0.,
                          velocity=velocities[idx], # if abs(velocities[idx]) > 1e-3 else 0,
                          # max_velocity=abs(velocities[idx]),
                          max_velocity=abs(max_velocities[idx]), # TODO: max_velocity and velocity==0 cause issues
                          position_gain=10, velocity_scale=None, max_force=None)
        else:
            velocity_control_joint(robot, joint=joint, velocity=velocities[idx],
                                   max_velocity=abs(max_velocities[idx]),
                                   position_gain=None, velocity_scale=None, max_force=None)

def control_state(robot, joints, target_positions, target_velocities=None, position_tol=INF, velocity_tol=INF,
                  max_velocities=None, verbose=True): # TODO: max_accelerations
    if target_velocities is None:
        target_velocities = np.zeros(len(joints))
    if max_velocities is None:
        max_velocities = get_max_velocities(robot, joints)
    assert (max_velocities > 0).all()
    max_velocity_control_joints(robot, joints, positions=target_positions, velocities=target_velocities,
                                max_velocities=max_velocities)
    for i in irange(INF):
        current_positions = np.array(get_joint_positions(robot, joints))
        position_error = get_distance(current_positions, target_positions, norm=INF)
        current_velocities = np.array(get_joint_velocities(robot, joints))
        velocity_error = get_distance(current_velocities, target_velocities, norm=INF)
        if verbose:
            # print('Positions: {} | Target positions: {}'.format(current_positions.round(N_DIGITS), target_positions.round(N_DIGITS)))
            # print('Velocities: {} | Target velocities: {}'.format(current_velocities.round(N_DIGITS), target_velocities.round(N_DIGITS)))
            print('Step: {} | Position error: {:.3f}/{:.3f} | Velocity error: {:.3f}/{:.3f}'.format(
                i, position_error, position_tol, velocity_error, velocity_tol))
        # TODO: draw the tolerance interval
        if (position_error <= position_tol) and (velocity_error <= velocity_tol):
            return # TODO: declare success or failure by yielding or throwing an exception
        yield i

def follow_curve(robot, joints, curve, goal_t=None, time_step=None, max_velocities=MAX_VELOCITIES, **kwargs):
    if goal_t is None:
        goal_t = curve.x[-1]
    if time_step is None:
        time_step = 10*get_time_step()
    #distance_fn = get_distance_fn(robot, joints, weights=None, norm=2)
    distance_fn = get_duration_fn(robot, joints, velocities=max_velocities, norm=INF) # get_distance
    positions = np.array(get_joint_positions(robot, joints))
    closest_dist, closest_t = find_closest(positions, curve, t_range=(curve.x[0], goal_t), max_time=1e-1,
                                           max_iterations=INF, distance_fn=distance_fn, verbose=True)
    print('Closest dist: {:.3f} | Closest time: {:.3f}'.format(closest_dist, closest_t))
    target_t = closest_t
    # TODO: condition based on closest_dist
    while True:
        print('\nTarget time: {:.3f} | Goal time: {}'.format(target_t, goal_t))
        target_positions = curve(target_t)
        target_velocities = curve(target_t, nu=1) # TODO: draw the velocity
        #print('Positions: {} | Velocities: {}'.format(target_positions, target_velocities))
        handles = draw_waypoint(target_positions)
        is_goal = (target_t == goal_t)
        position_tol = 1e-2 if is_goal else 1e-2
        for output in control_state(robot, joints, target_positions=target_positions, target_velocities=target_velocities,
                                    position_tol=position_tol, velocity_tol=INF, max_velocities=max_velocities, **kwargs):
            yield output
        remove_handles(handles)
        target_t = min(goal_t, target_t + time_step)
        if is_goal:
            break

##################################################

def follow_curve_old(robot, joints, curve, goal_t=None):
    # TODO: unify with /Users/caelan/Programs/open-world-tamp/open_world/simulation/control.py
    if goal_t is None:
        goal_t = curve.x[-1]
    time_step = get_time_step()
    target_step = 10*time_step
    #distance_fn = get_distance_fn(robot, joints, weights=None, norm=2)
    distance_fn = get_duration_fn(robot, joints, velocities=MAX_VELOCITIES, norm=INF)
    for i in irange(INF):
        # if (i % 10) != 0:
        #     continue
        current_p = np.array(get_joint_positions(robot, joints))
        current_v = np.array(get_joint_velocities(robot, joints))
        goal_dist = distance_fn(current_p, curve(goal_t))
        print('Positions: {} | Velocities: {} | Goal distance: {:.3f}'.format(
            current_p.round(N_DIGITS), current_v.round(N_DIGITS), goal_dist))
        if goal_dist < 1e-2:
            return True

        # _, connection = mpc(current_p, current_v, curve, v_max=MAX_VELOCITIES, a_max=MAX_ACCELERATIONS,
        #                     dt_max=1e-1, max_time=1e-1)
        # assert connection is not None
        # target_t = 0.5*connection.x[-1]
        # target_p = connection(target_t)
        # target_v = connection(target_t, nu=1)
        # #print(target_p)

        closest_dist, closest_t = find_closest(current_p, curve, t_range=None, max_time=1e-2,
                                               max_iterations=INF, distance_fn=distance_fn, verbose=True)
        target_t = min(closest_t + target_step, curve.x[-1])
        target_p = curve(target_t)
        #target_v = curve(target_t, nu=1)
        target_v = curve(closest_t, nu=1)

        #target_v = MAX_VELOCITIES
        #target_v = INF*np.zeros(len(joints))

        handles = draw_waypoint(target_p)
        #times, confs = time_discretize_curve(curve, verbose=False, resolution=resolutions)  # max_velocities=v_max,

        # set_joint_positions(robot, joints, target_p)
        max_velocity_control_joints(robot, joints,
                                    positions=target_p,
                                    velocities=target_v,
                                    max_velocities=MAX_VELOCITIES)

        #next_t = closest_t + time_step
        #next_p = current_p + current_v*time_step
        yield target_t
        actual_p = np.array(get_joint_positions(robot, joints))
        actual_v = np.array(get_joint_velocities(robot, joints))
        next_p = current_p + actual_v*time_step
        print('Predicted: {} | Actual: {}'.format(next_p.round(N_DIGITS), actual_p.round(N_DIGITS)))
        remove_handles(handles)

##################################################

def simulate_curve(robot, joints, curve):
    #set_joint_positions(robot, joints, curve(random.uniform(curve.x[0], curve.x[-1])))
    wait_if_gui(message='Begin?')
    #controller = follow_curve_old(robot, joints, curve)
    controller = follow_curve(robot, joints, curve)
    for _ in controller:
        step_simulation()
        #wait_if_gui()
        #wait_for_duration(duration=time_step)
        #time.sleep(time_step)
    wait_if_gui(message='Finish?')

def step_curve(robot, joints, path, step_size=None):
    wait_if_gui(message='Begin?')
    for i, conf in enumerate(path):
        set_joint_positions(robot, joints, conf)
        if step_size is None:
            wait_if_gui(message='{}/{} Continue?'.format(i, len(path)))
        else:
            wait_for_duration(duration=step_size)
    wait_if_gui(message='Finish?')

##################################################

def iterate_path(robot, joints, path, simulate=True, step_size=None, resolutions=None, smooth=True, **kwargs): # 1e-2 | None
    if path is None:
        return
    path = adjust_path(robot, joints, path)
    with LockRenderer():
        handles = draw_path(path)

    waypoints = path
    #waypoints = waypoints_from_path(path)
    #curve = interpolate_path(robot, joints, waypoints, k=1, velocity_fraction=1) # TODO: no velocities in the URDF

    curve = solve_multi_linear(waypoints, v_max=MAX_VELOCITIES, a_max=MAX_ACCELERATIONS)
    _, path = time_discretize_curve(curve, verbose=False, resolution=resolutions) # max_velocities=v_max,
    print('Steps: {} | Start: {:.3f} | End: {:.3f} | Knots: {}'.format(
        len(path), curve.x[0], curve.x[-1], len(curve.x)))
    with LockRenderer():
        handles = draw_path(path)

    if smooth:
        # TODO: handle circular joints
        #curve_collision_fn = lambda *args, **kwargs: False
        curve_collision_fn = get_curve_collision_fn(robot, joints, resolutions=resolutions, **kwargs)
        with LockRenderer():
            with BodySaver(robot):
                curve = smooth_curve(curve, MAX_VELOCITIES, MAX_ACCELERATIONS, curve_collision_fn, max_time=5) #, curve_collision_fn=[])
        path = [conf for t, conf in sample_curve(curve, time_step=step_size)]
        print('Steps: {} | Start: {:.3f} | End: {:.3f} | Knots: {}'.format(
            len(path), curve.x[0], curve.x[-1], len(curve.x)))
        with LockRenderer():
            handles = draw_path(path)

    if simulate:
        simulate_curve(robot, joints, curve)
    else:
        path = [conf for t, conf in sample_curve(curve, time_step=step_size)]
        step_curve(robot, joints, path, step_size=step_size)

##################################################

def test_aabb(robot):
    base_link = link_from_name(robot, BASE_LINK_NAME)
    region_aabb = AABB(lower=-np.ones(3), upper=+np.ones(3))
    draw_aabb(region_aabb)

    # bodies = get_bodies_in_region(region_aabb)
    # print(len(bodies), bodies)
    # for body in get_bodies():
    #     set_pose(body, Pose())

    #step_simulation()  # Need to call before get_bodies_in_region
    #update_scene()
    for i in range(3):
        with timer(message='{:f}'):
            bodies = get_bodies_in_region(region_aabb) # This does cache some info
        print(i, len(bodies), bodies)
    # https://github.com/bulletphysics/bullet3/search?q=broadphase
    # https://github.com/bulletphysics/bullet3/search?p=1&q=getCachedOverlappingObjects&type=&utf8=%E2%9C%93
    # https://andysomogyi.github.io/mechanica/bullet.html
    # http://www.cs.kent.edu/~ruttan/GameEngines/lectures/Bullet_User_Manual

    aabb = get_aabb(robot)
    # aabb = get_subtree_aabb(robot, base_link)
    print(aabb)
    draw_aabb(aabb)

    for link in [BASE_LINK, base_link]:
        print(link, get_collision_data(robot, link), pairwise_link_collision(robot, link, robot, link))

def test_caching(robot, obstacles):
    with timer(message='{:f}'):
        #update_scene() # 5.19752502441e-05
        step_simulation() # 0.000210046768188
    with timer(message='{:f}'):
        #print(get_aabb(robot, link=None, only_collision=True))
        print(contact_collision()) # 2.50339508057e-05
    for _ in range(3):
        with timer(message='{:f}'):
            #print(get_aabb(robot, link=None, only_collision=True)) # Recomputes each time
            print(contact_collision()) # 1.69277191162e-05

    print()
    obstacle = obstacles[-1]
    #for link in get_all_links(robot):
    #    set_collision_pair_mask(robot, obstacle, link1=link, enable=False) # Doesn't seem to affect pairwise_collision
    with timer('{:f}'):
        print(pairwise_collision(robot, obstacle)) # 0.000031
    links = get_all_links(robot)
    links = [link for link in get_all_links(robot) if can_collide(robot, link)]
    #links = randomize(links)
    with timer('{:f}'):
        print(any(pairwise_collision(robot, obstacle, link1=link) for link in links # 0.000179
                  ))
                  #if aabb_overlap(get_aabb(robot, link), get_aabb(obstacles[-1]))))
                  #if can_collide(robot, link)))
    with timer('{:f}'):
        print(pairwise_collision(robot, obstacle))

##################################################

def main():
    np.set_printoptions(precision=N_DIGITS, suppress=True, threshold=3)  # , edgeitems=1) #, linewidth=1000)
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--algorithm', default='rrt_connect', # choices=[],
                        help='The motion planning algorithm to use.')
    parser.add_argument('-c', '--cfree', action='store_true',
                        help='When enabled, disables collision checking.')
    # parser.add_argument('-p', '--problem', default='test_pour', choices=sorted(problem_fn_from_name),
    #                     help='The name of the problem to solve.')
    parser.add_argument('--holonomic', action='store_true', # '-h',
                        help='')
    parser.add_argument('-l', '--lock', action='store_false',
                        help='')
    parser.add_argument('-s', '--seed', default=None, type=int, # None | 1
                        help='The random seed to use.')
    parser.add_argument('-n', '--num', default=10, type=int,
                        help='The number of obstacles.')
    parser.add_argument('-o', '--orientation', action='store_true',
                        help='')
    parser.add_argument('-v', '--viewer', action='store_false',
                        help='')
    args = parser.parse_args()
    connect(use_gui=args.viewer)
    #set_aabb_buffer(buffer=1e-3)
    #set_separating_axis_collisions()

    #seed = 0
    #seed = time.time()
    seed = args.seed
    if seed is None:
        seed = random.randint(0, 10**3-1)
    print('Seed:', seed)
    set_random_seed(seed=seed) # None: 2147483648 = 2**31
    set_numpy_seed(seed=seed)
    #print('Random seed:', get_random_seed(), random.random())
    #print('Numpy seed:', get_numpy_seed(), np.random.random())

    #########################

    robot, base_limits, goal_conf, obstacles = problem1(n_obstacles=args.num)
    custom_limits = create_custom_base_limits(robot, base_limits)
    base_joints = joints_from_names(robot, BASE_JOINTS)

    draw_base_limits(base_limits)
    # draw_pose(get_link_pose(robot, base_link), length=0.5)
    start_conf = get_joint_positions(robot, base_joints)
    for conf in [start_conf, goal_conf]:
        draw_waypoint(conf)

    #resolutions = None
    #resolutions = np.array([0.05, 0.05, math.radians(10)])
    resolutions = 1.*DEFAULT_RESOLUTION*np.ones(len(base_joints))
    plan_joints = base_joints[:2] if not args.orientation else base_joints
    holonomic = args.holonomic or (len(plan_joints) != 3)

    if args.cfree:
        obstacles = []
    # for obstacle in obstacles:
    #     draw_aabb(get_aabb(obstacle)) # Updates automatically
    #set_all_static() # Doesn't seem to affect

    #test_aabb(robot)
    #test_caching(robot, obstacles)
    #return

    #########################

    saver = WorldSaver()
    start_time = time.time()
    profiler = Profiler(field='cumtime', num=50) # tottime | cumtime | None
    profiler.save()
    with LockRenderer(lock=args.lock):
        # TODO: draw the search tree
        path = plan_motion(robot, plan_joints, goal_conf[:len(plan_joints)], holonomic=holonomic,
                           obstacles=obstacles, self_collisions=False,
                           custom_limits=custom_limits, resolutions=resolutions[:len(plan_joints)],
                           use_aabb=True, cache=True, max_distance=MIN_PROXIMITY,
                           algorithm=args.algorithm, restarts=5, max_iterations=50, smooth=20) # 20 | None
        saver.restore()
    #wait_for_duration(duration=1e-3)
    profiler.restore()

    #########################

    solved = path is not None
    length = INF if path is None else len(path)
    cost = compute_cost(robot, base_joints, path, resolutions=resolutions[:len(plan_joints)])
    print('Solved: {} | Length: {} | Cost: {:.3f} | Runtime: {:.3f} sec'.format(
        solved, length, cost, elapsed_time(start_time)))
    if path is None:
        wait_if_gui()
        disconnect()
        return

    path = extract_full_path(robot, plan_joints, path, base_joints)
    iterate_path(robot, base_joints, path, step_size=2e-2, custom_limits=custom_limits, resolutions=resolutions,
                 obstacles=obstacles, self_collisions=False, max_distance=MIN_PROXIMITY)
    disconnect()

if __name__ == '__main__':
    main()
