import math
import numpy as np

from pybullet_tools.utils import clip, INF, \
    waypoints_from_path, adjust_path, get_difference, get_pairs, get_max_velocities, get_duration_fn, wait_if_gui

#ARM_SPEED = 0.15*np.pi # radians / sec
ARM_SPEED = 0.2 # percent
DEFAULT_SPEED_FRACTION = 0.3

################################################################################

def ensure_increasing(path, time_from_starts):
    assert len(path) == len(time_from_starts)
    for i in reversed(range(1, len(path))):
        if time_from_starts[i-1] == time_from_starts[i]:
            path.pop(i)
            time_from_starts.pop(i)

def decompose_into_paths(joints, path):
    current_path = []
    joint_sequence = []
    path_sequence = []
    for q1, q2 in get_pairs(path):
        # Zero velocities aren't enforced otherwise
        indices, = np.nonzero(get_difference(q1, q2))
        current_joints = tuple(joints[j] for j in indices)
        if not joint_sequence or (current_joints != joint_sequence[-1]):
            if current_path:
                path_sequence.append(current_path)
            joint_sequence.append(current_joints)
            current_path = [tuple(q1[j] for j in indices)]
        current_path.append(tuple(q2[j] for j in indices))
    if current_path:
        path_sequence.append(current_path)
    return zip(joint_sequence, path_sequence)

################################################################################

# TODO: retain based on the end effector velocity

def instantaneous_retime_path(robot, joints, path, speed=ARM_SPEED):
    duration_fn = get_duration_fn(robot, joints) # get_distance_fn
    mid_durations = [duration_fn(*pair) for pair in get_pairs(path)]
    durations = [0.] + mid_durations
    time_from_starts = np.cumsum(durations) / speed
    return time_from_starts

def slow_trajectory(robot, joints, path, min_fraction=0.1, ramp_duration=1.0, **kwargs):
    """
    :param robot:
    :param joints:
    :param path:
    :param min_fraction: percentage
    :param ramp_duration: seconds
    :param kwargs:
    :return:
    """
    time_from_starts = instantaneous_retime_path(robot, joints, path, **kwargs)
    mid_times = [np.average(pair) for pair in get_pairs(time_from_starts)]
    mid_durations = [t2 - t1 for t1, t2 in get_pairs(time_from_starts)]
    new_time_from_starts = [0.]
    for mid_time, mid_duration in zip(mid_times, mid_durations):
        time_from_start = mid_time - time_from_starts[0]
        up_fraction = clip(time_from_start / ramp_duration, min_value=min_fraction, max_value=1.)
        time_from_end = time_from_starts[-1] - mid_time
        down_fraction = clip(time_from_end / ramp_duration, min_value=min_fraction, max_value=1.)
        new_fraction = min(up_fraction, down_fraction)
        new_duration = mid_duration / new_fraction
        #print(new_time_from_starts[-1], up_fraction, down_fraction, new_duration)
        new_time_from_starts.append(new_time_from_starts[-1] + new_duration)
    # print(time_from_starts)
    # print(new_time_from_starts)
    # wait_if_gui('Continue?)
    # time_from_starts = new_time_from_starts
    return new_time_from_starts

#def acceleration_limits(robot, joints, path, speed=ARM_SPEED, **kwargs):
#    # TODO: multiple bodies (such as drawer)
#    # The drawers do actually have velocity limits
#    fraction = 0.25
#    duration_fn = get_duration_fn(robot, joints)
#    max_velocities = speed * np.array(get_max_velocities(robot, joints))
#    max_accelerations = 2*fraction*max_velocities # TODO: fraction
#    difference_fn = get_difference_fn(robot, joints)
#    differences1 = [difference_fn(q2, q1) for q1, q2 in zip(path[:-1], path[1:])]
#    differences2 = [np.array(d2) - np.array(d1) for d1, d2 in zip(differences1[:-1], differences1[1:])] # TODO: circular case

################################################################################

def compute_ramp_duration(distance, acceleration, duration):
    discriminant = max(0, math.pow(duration * acceleration, 2) - 4 * distance * acceleration)
    velocity = 0.5 * (duration * acceleration - math.sqrt(discriminant))  # +/-
    #assert velocity <= max_velocity
    ramp_time = velocity / acceleration
    predicted_distance = velocity * (duration - 2 * ramp_time) + acceleration * math.pow(ramp_time, 2)
    assert abs(distance - predicted_distance) < 1e-6
    return ramp_time

def compute_position(ramp_time, max_duration, acceleration, t):
    velocity = acceleration * ramp_time
    max_time = max_duration - 2 * ramp_time
    t1 = clip(t, 0, ramp_time)
    t2 = clip(t - ramp_time, 0, max_time)
    t3 = clip(t - ramp_time - max_time, 0, ramp_time)
    #assert t1 + t2 + t3 == t
    return 0.5 * acceleration * math.pow(t1, 2) + velocity * t2 + \
           velocity * t3 - 0.5 * acceleration * math.pow(t3, 2)

def add_ramp_waypoints(differences, accelerations, q1, duration, sample_step, waypoints, time_from_starts):
    dim = len(q1)
    distances = np.abs(differences)
    time_from_start = time_from_starts[-1]

    ramp_durations = [compute_ramp_duration(distances[idx], accelerations[idx], duration)
                      for idx in range(dim)]
    directions = np.sign(differences)
    for t in np.arange(sample_step, duration, sample_step):
        positions = []
        for idx in range(dim):
            distance = compute_position(ramp_durations[idx], duration, accelerations[idx], t)
            positions.append(q1[idx] + directions[idx] * distance)
        waypoints.append(positions)
        time_from_starts.append(time_from_start + t)
    return waypoints, time_from_starts

################################################################################

def compute_min_duration(distance, max_velocity, acceleration):
    if distance == 0:
        return 0
    max_ramp_duration = max_velocity / acceleration
    if acceleration == INF:
        #return distance / max_velocity
        ramp_distance = 0.
    else:
        ramp_distance = 0.5 * acceleration * math.pow(max_ramp_duration, 2)
    remaining_distance = distance - 2 * ramp_distance
    if 0 <= remaining_distance:  # zero acceleration
        remaining_time = remaining_distance / max_velocity
        total_time = 2 * max_ramp_duration + remaining_time
    else:
        half_time = np.sqrt(distance / acceleration)
        total_time = 2 * half_time
    return total_time

def ramp_retime_path(path, max_velocities, acceleration_fraction=INF, sample_step=None):
    """
    :param path:
    :param max_velocities:
    :param acceleration_fraction: fraction of velocity_fraction*max_velocity per second
    :param sample_step:
    :return:
    """
    assert np.all(max_velocities)
    accelerations = max_velocities * acceleration_fraction
    dim = len(max_velocities)
    #difference_fn = get_difference_fn(robot, joints)
    # TODO: more fine grain when moving longer distances

    # Assuming instant changes in accelerations
    waypoints = [path[0]]
    time_from_starts = [0.]
    for q1, q2 in get_pairs(path):
        differences = get_difference(q1, q2) # assumes not circular anymore
        #differences = difference_fn(q1, q2)
        distances = np.abs(differences)
        duration = max([compute_min_duration(distances[idx], max_velocities[idx], accelerations[idx])
                        for idx in range(dim)] + [0.])
        time_from_start = time_from_starts[-1]
        if sample_step is not None:
            waypoints, time_from_starts = add_ramp_waypoints(differences, accelerations, q1, duration, sample_step,
                                                             waypoints, time_from_starts)
        waypoints.append(q2)
        time_from_starts.append(time_from_start + duration)
    return waypoints, time_from_starts

def retime_trajectory(robot, joints, path, only_waypoints=False,
                      velocity_fraction=DEFAULT_SPEED_FRACTION, **kwargs):
    """
    :param robot:
    :param joints:
    :param path:
    :param velocity_fraction: fraction of max_velocity
    :return:
    """
    path = adjust_path(robot, joints, path)
    if only_waypoints:
        path = waypoints_from_path(path)
    max_velocities = velocity_fraction * np.array(get_max_velocities(robot, joints))
    return ramp_retime_path(path, max_velocities, **kwargs)

################################################################################

def approximate_spline(time_from_starts, path, k=3, approx=INF):
    from scipy.interpolate import make_interp_spline, make_lsq_spline
    x = time_from_starts
    if approx == INF:
        positions = make_interp_spline(time_from_starts, path, k=k, t=None, bc_type='clamped')
        positions.x = positions.t[positions.k:-positions.k]
    else:
        # TODO: approximation near the endpoints
        # approx = min(approx, len(x) - 2*k)
        assert approx <= len(x) - 2 * k
        t = np.r_[(x[0],) * (k + 1),
                  # np.linspace(x[0]+1e-3, x[-1]-1e-3, num=approx, endpoint=True),
                  np.linspace(x[0], x[-1], num=2 + approx, endpoint=True)[1:-1],
                  (x[-1],) * (k + 1)]
        # t = positions.t # Need to slice
        # w = np.zeros(...)
        w = None
        positions = make_lsq_spline(x, path, t, k=k, w=w)
    positions.x = positions.t[positions.k:-positions.k]
    return positions

def interpolate_path(robot, joints, path, velocity_fraction=DEFAULT_SPEED_FRACTION,
                     k=1, bspline=False, dump=False, **kwargs):
    from scipy.interpolate import CubicSpline, interp1d
    #from scipy.interpolate import CubicHermiteSpline, KroghInterpolator
    # https://scikit-learn.org/stable/auto_examples/linear_model/plot_polynomial_interpolation.html
    # TODO: local search to retime by adding or removing waypoints
    # TODO: iteratively increase spacing until velocity / accelerations meets limits
    # https://docs.scipy.org/doc/scipy/reference/tutorial/interpolate.html
    # Waypoints are followed perfectly, twice continuously differentiable
    # TODO: https://pythonrobotics.readthedocs.io/en/latest/modules/path_tracking.html#mpc-modeling
    path, time_from_starts = retime_trajectory(robot, joints, path, velocity_fraction=velocity_fraction, sample_step=None)
    if k == 3:
        if bspline:
            positions = approximate_spline(time_from_starts, path, k=k, **kwargs)
        else:
            # bc_type= clamped | natural | ((1, 0), (1, 0))
            positions = CubicSpline(time_from_starts, path, bc_type='clamped', extrapolate=False)
    else:
        kinds = {1: 'linear', 2: 'quadratic', 3: 'cubic'} # slinear
        positions = interp1d(time_from_starts, path, kind=kinds[k], axis=0, assume_sorted=True)

    if not dump:
        return positions
    # TODO: only if CubicSpline
    velocities = positions.derivative()
    accelerations = positions.derivative()
    for i, t in enumerate(positions.x):
        print(i, round(t, 3), positions(t), velocities(t), accelerations(t))
    # TODO: compose piecewise functions
    # TODO: ramp up and ramp down path
    # TODO: quadratic interpolation between endpoints
    return positions


def sample_curve(positions_curve, time_step=1e-2):
    start_time = positions_curve.x[0]
    end_time = positions_curve.x[-1]
    times = np.append(np.arange(start_time, end_time, step=time_step), [end_time])
    for t in times:
        q = positions_curve(t)
        yield t, q
