import math
import numpy as np

from pybullet_tools.utils import get_distance_fn, get_joint_name, clip, get_max_velocity, get_difference_fn, INF, \
    waypoints_from_path, adjust_path, get_difference, get_pairs

#ARM_SPEED = 0.15*np.pi # radians / sec
ARM_SPEED = 0.2 # percent
DEFAULT_SPEED_FRACTION = 0.3

################################################################################

def get_duration_fn(body, joints, velocities=None, norm=INF):
    if velocities is None:
        velocities = np.array([get_max_velocity(body, joint) for joint in joints])
    difference_fn = get_difference_fn(body, joints)
    def fn(q1, q2):
        distance = np.array(difference_fn(q2, q1))
        duration = np.divide(distance, velocities)
        return np.linalg.norm(duration, ord=norm)
    return fn

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
    #duration_fn = get_distance_fn(robot, joints)
    duration_fn = get_duration_fn(robot, joints)
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
    # raw_input('Continue?)
    # time_from_starts = new_time_from_starts
    return new_time_from_starts

#def acceleration_limits(robot, joints, path, speed=ARM_SPEED, **kwargs):
#    # TODO: multiple bodies (such as drawer)
#    # The drawers do actually have velocity limits
#    fraction = 0.25
#    duration_fn = get_duration_fn(robot, joints)
#    max_velocities = speed * np.array([get_max_velocity(robot, joint) for joint in joints])
#    max_accelerations = 2*fraction*max_velocities # TODO: fraction
#    difference_fn = get_difference_fn(robot, joints)
#    differences1 = [difference_fn(q2, q1) for q1, q2 in zip(path[:-1], path[1:])]
#    differences2 = [np.array(d2) - np.array(d1) for d1, d2 in zip(differences1[:-1], differences1[1:])] # TODO: circular case

################################################################################

def compute_min_duration(distance, max_velocity, acceleration):
    if distance == 0:
        return 0
    max_ramp_duration = max_velocity / acceleration
    ramp_distance = 0.5 * acceleration * math.pow(max_ramp_duration, 2)
    remaining_distance = distance - 2 * ramp_distance
    if 0 <= remaining_distance:  # zero acceleration
        remaining_time = remaining_distance / max_velocity
        total_time = 2 * max_ramp_duration + remaining_time
    else:
        half_time = np.sqrt(distance / acceleration)
        total_time = 2 * half_time
    return total_time

def compute_ramp_duration(distance, max_velocity, acceleration, duration):
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
           (velocity * t3 - 0.5 * acceleration * math.pow(t3, 2))

def ramp_retime_path(path, max_velocities, acceleration_fraction=1.5, sample_step=None):
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
        duration = 0
        for idx in range(dim):
            total_time = compute_min_duration(distances[idx], max_velocities[idx], accelerations[idx])
            duration = max(duration, total_time)

        time_from_start = time_from_starts[-1]
        if sample_step is not None:
            ramp_durations = [compute_ramp_duration(distances[idx], max_velocities[idx], accelerations[idx], duration)
                              for idx in range(dim)]
            directions = np.sign(differences)
            for t in np.arange(sample_step, duration, sample_step):
                positions = []
                for idx in range(dim):
                    distance = compute_position(ramp_durations[idx], duration, accelerations[idx], t)
                    positions.append(q1[idx] + directions[idx] * distance)
                waypoints.append(positions)
                time_from_starts.append(time_from_start + t)
        waypoints.append(q2)
        time_from_starts.append(time_from_start + duration)
    return waypoints, time_from_starts

def retime_trajectory(robot, joints, path, velocity_fraction=DEFAULT_SPEED_FRACTION, **kwargs):
    """
    :param robot:
    :param joints:
    :param path:
    :param velocity_fraction: fraction of max_velocity
    :return:
    """
    path = adjust_path(robot, joints, path)
    path = waypoints_from_path(path)
    max_velocities = velocity_fraction * np.array([get_max_velocity(robot, joint) for joint in joints])
    waypoints, time_from_starts = ramp_retime_path(path, max_velocities, **kwargs)
    return waypoints, time_from_starts

################################################################################

def interpolate_path(robot, joints, path):
    from scipy.interpolate import CubicSpline
    # TODO: iteratively increase spacing until velocity / accelerations meets limits
    # https://docs.scipy.org/doc/scipy/reference/tutorial/interpolate.html
    # Waypoints are followed perfectly, twice continuously differentiable
    path, time_from_starts = retime_trajectory(robot, joints, path, sample_step=None)
    # positions_curve = interp1d(time_from_starts, path, kind='linear', axis=0, assume_sorted=True)
    positions_curve = CubicSpline(time_from_starts, path, bc_type='clamped',  # clamped | natural
                                  extrapolate=False)  # bc_type=((1, 0), (1, 0))
    return positions_curve
