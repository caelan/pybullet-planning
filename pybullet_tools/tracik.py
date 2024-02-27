import itertools
import math
import os.path
import time

import numpy as np
from tracikpy import TracIKSolver

from pybullet_planning.pybullet_tools.utils import Pose, multiply, invert, tform_from_pose, get_model_info, BASE_LINK, \
    get_link_name, link_from_name, get_joint_name, joint_from_name, parent_link_from_joint, joints_from_names, \
    links_from_names, get_link_pose, draw_pose, set_joint_positions, get_joint_positions, get_joint_limits, \
    CIRCULAR_LIMITS, UNBOUNDED_LIMITS, get_custom_limits, irange, INF, PI, elapsed_time, is_circular, get_distance, \
    all_between, get_difference_fn, Saver


class LimitsSaver(Saver):
    def __init__(self, ik_solver): # joint_limits=None
        self.ik_solver = ik_solver
        self.joint_limits = ik_solver.joint_limits
        # if joint_limits is not None:
        #     self.ik_solver.set_joint_limits(*joint_limits)
    def restore(self):
        self.ik_solver.set_joint_limits(*self.joint_limits)

class IKSolver(object): # TODO: rename?
    def __init__(self, body, tool_link, first_joint=None, tool_offset=Pose(), custom_limits={},
                 seed=None, speed=True, max_time=5e-3, error=1e-5): #, **kwargs):
        # TODO: unify with my other tracikpy wrappers
        self.body = body
        if isinstance(tool_link, str):
            tool_link = link_from_name(body, tool_link)
        self.tool_link = tool_link
        if first_joint is None:
            self.base_link = BASE_LINK
        else:
            if isinstance(first_joint, str):
                first_joint = joint_from_name(body, first_joint)
            self.base_link = parent_link_from_joint(body, first_joint)
        # joints = get_joint_ancestors(body, self.tool_link)[1:] # get_link_ancestors
        # movable_joints = prune_fixed_joints(body, joints)
        # print([get_joint_name(body, joint) for joint in movable_joints])

        # TODO: Distance doesn't make sense when circular limits
        urdf_info = get_model_info(body)
        self.urdf_path = os.path.abspath(urdf_info.path) # self.ik_solver._urdf_string
        self.ik_solver = TracIKSolver(
            urdf_file=self.urdf_path,
            base_link=get_link_name(self.body, self.base_link),
            tip_link=get_link_name(self.body, self.tool_link),
            timeout=max_time, epsilon=error,
            solve_type='Speed' if speed else 'Distance', # Manipulation1 | Manipulation2
        )
        assert self.ik_solver.joint_names

        self.circular_limits = list(get_custom_limits(
            self.body, self.joints, custom_limits=custom_limits, circular_limits=CIRCULAR_LIMITS))
        self.unbounded_limits = list(get_custom_limits(
            self.body, self.joints, custom_limits=custom_limits, circular_limits=UNBOUNDED_LIMITS))
        self.circular_joints = [is_circular(self.body, joint) for joint in self.joints]
        self.default_bound = [PI if circular else INF for joint, circular in zip(self.joints, self.circular_joints)]
        self.difference_fn = get_difference_fn(self.body, self.joints)
        self.reset_limits()

        self.tool_offset = tool_offset # None
        self.random_generator = np.random.RandomState(seed)
        self.solutions = []
        self.handles = []
    @property
    def robot(self):
        return self.body
    @property
    def base_name(self):
        return self.ik_solver.base_link
    @property
    def tool_name(self):
        return self.ik_solver.tip_link
    @property
    def link_names(self):
        return self.ik_solver.link_names
    @property
    def joint_names(self):
        return self.ik_solver.joint_names
    @property
    def links(self):
        return links_from_names(self.body, self.link_names)
    @property
    def joints(self):
        return joints_from_names(self.body, self.joint_names)
    @property
    def dofs(self):
        return len(self.joints)
    @property
    def joint_limits(self):
        return self.ik_solver.joint_limits
    @property
    def lower_limits(self):
        lower, _ = self.joint_limits
        return lower
    @property
    def upper_limits(self):
        _, upper = self.joint_limits
        return upper
    @property
    def last_solution(self):
        if not self.solutions:
            return None
        pose, conf = self.solutions[-1]
        return conf
    @property
    def reference_conf(self): # TODO: set_reference_conf
        return np.average(self.joint_limits, axis=0)

    def get_link_name(self, link):
        if isinstance(link, str):
           return link
        return get_link_name(self.body, link)
    def get_joint_name(self, joint):
        if isinstance(joint, str):
           return joint
        return get_joint_name(self.body, joint)
    def get_link_pose(self, link):
        return get_link_pose(self.body, link)
    def get_base_pose(self):
        return self.get_link_pose(self.base_link)
    def get_tool_pose(self):
        # TODO: self.ik_solver.fk()
        return self.get_link_pose(self.tool_link)
    def world_from_base(self, pose):
        base_pose = self.get_base_pose()
        return multiply(base_pose, pose)
    def base_from_world(self, pose):
        base_pose = self.get_base_pose()
        return multiply(invert(base_pose), pose)
    def draw_pose(self, pose=None, **kwargs):
        if pose is None:
            pose = self.get_tool_pose()
        self.handles.extend(draw_pose(pose, **kwargs))

    def saver(self):
        return LimitsSaver(self)
    def get_conf(self):
        return get_joint_positions(self.body, self.joints)
    def set_conf(self, conf):
        assert conf is not None
        set_joint_positions(self.body, self.joints, conf)
    def sample_conf(self):
        # TODO: truncated Gaussian
        return self.random_generator.uniform(*self.joint_limits)
    def reset_limits(self):
        # TODO: limits saver
        self.set_joint_limits(*self.circular_limits)
    def set_joint_limits(self, lower, upper): # TODO: pass a pair?
        # TODO: uses -3.40282347e+38 if not set
        lower_limits, upper_limits = self.unbounded_limits
        lower = np.maximum(lower, lower_limits)
        upper = np.minimum(upper, upper_limits)
        self.ik_solver.joint_limits = (lower, upper)
    def set_nearby_limits(self, conf, bound=None):
        if bound is None:
            bound = self.default_bound
        bound = np.minimum(bound, self.default_bound)
        lower = np.array(conf) - bound
        upper = np.array(conf) + bound
        self.set_joint_limits(lower, upper)
        return lower, upper
    def within_limits(self, conf):
        return all_between(self.lower_limits, conf, self.upper_limits)
    def adjust_conf(self, target_conf, reference_conf): # wrap_conf
        if (target_conf is None) or (reference_conf is None):
            return target_conf
        # from pybullet_planning.pybullet_tools.utils import adjust_path
        difference = self.difference_fn(target_conf, reference_conf)
        return reference_conf + difference

    def solve(self, tool_pose, seed_conf=None, pos_tolerance=1e-5, ori_tolerance=math.radians(5e-2)):
        # TODO: convert from another frame into tool frame?
        pose = self.base_from_world(tool_pose)
        tform = tform_from_pose(pose)
        #if seed_conf is None: # TODO: will use np.random.default_rng()
        #    seed_conf = self.reference_conf
        # if seed_conf is not None:
        #     self.set_nearby_limits(seed_conf)
        bx, by, bz = pos_tolerance * np.ones(3)
        brx, bry, brz = ori_tolerance * np.ones(3)
        conf = self.ik_solver.ik(tform, qinit=seed_conf, bx=bx, by=by, bz=bz, brx=brx, bry=bry, brz=brz)
        # TODO: ignore limits potentially for circular joints
        # if (seed_conf is not None) and (conf is not None):
        #     conf = self.adjust_conf(conf, seed_conf)
        # assert (conf is None) or self.within_limits(conf)
        self.solutions.append((pose, conf))
        # self.reset_limits()
        return conf
    def solve_current(self, tool_pose, **kwargs): # solve_closest
        return self.solve(tool_pose, seed_conf=self.get_conf(), **kwargs)
    def solve_randomized(self, tool_pose, **kwargs):
        return self.solve(tool_pose, seed_conf=self.sample_conf(), **kwargs)
    def solve_reference(self, tool_pose, **kwargs):
        return self.solve(tool_pose, seed_conf=self.reference_conf, **kwargs)
    def solve_warm(self, tool_pose, **kwargs):
        return self.solve(tool_pose, seed_conf=self.last_solution, **kwargs)
    def solve_multiple(self, tool_pose, seed_conf=None, max_attempts=3, max_failures=INF,
                       max_time=INF, max_solutions=1, bound_discount=None, weights=None, verbose=False, **kwargs):
        # TODO: warm start from prior solution
        start_time = time.time()
        if weights is None:
            weights = np.ones(self.dofs)
        if seed_conf is None:
            bound_discount = None
        nearby_conf = seed_conf
        best_distance = INF
        failures = 0
        solutions = []
        for attempt in irange(max_attempts):
            if (elapsed_time(start_time) > max_time) or (len(solutions) > max_solutions):
                break
            if bound_discount is not None:
                bound = bound_discount * best_distance * np.reciprocal(weights)
                self.set_nearby_limits(nearby_conf, bound=bound)
            if (attempt != 0) or (seed_conf is None):
                # TODO: modify a subset of the degrees of freedom
                seed_conf = self.sample_conf()
            conf = self.solve(tool_pose, seed_conf=seed_conf, **kwargs)

            if conf is None:
                failures += 1
                if failures > max_failures:
                    break
                continue
            failures = 0
            distance = INF
            if bound_discount is not None:
                difference = self.difference_fn(conf, nearby_conf)
                # distance = np.linalg.norm(difference, ord=INF)
                distances = np.multiply(weights, np.absolute(difference))
                index = np.argmax(distances)
                distance = distances[index]
                # if distance < best_distance:
                best_distance = min(distance, best_distance)
                if verbose:
                    print(f'Iteration: {attempt}/{max_attempts} | Index: {index} | '
                          f'Current: {distance:.3f} | Best: {best_distance:.3f} | '
                          f'Elapsed: {elapsed_time(start_time):.3f}')
            solutions.append((conf, distance))
        self.reset_limits()
        solutions.sort(key=lambda p: p[1], reverse=False)
        return [conf for conf, _ in solutions]
    def solve_restart(self, tool_pose, **kwargs):
        solutions = self.solve_multiple(tool_pose, max_solutions=1, **kwargs)
        if not solutions:
            return None
        return solutions[0]
    def solve_distance(self, tool_pose, seed_conf, max_attempts=INF, max_time=0.1,
                       max_failures=0, bound_discount=0.95, **kwargs):
        # TODO: shrink all (L-inf) vs one coordinate
        return self.solve_restart(tool_pose, seed_conf=seed_conf, max_attempts=max_attempts, max_time=max_time,
                                  max_solutions=INF, max_failures=max_failures, bound_discount=bound_discount, **kwargs)
    def generate(self, tool_pose, seed_conf=None, joint_limits=None, include_failures=True, **kwargs):
        #start_time = time.time()
        for i in itertools.count():
            #print(elapsed_time(start_time))
            with self.saver():
                if joint_limits is not None:
                    self.set_joint_limits(*joint_limits)
                if (i != 0) or (seed_conf is None):
                    seed_conf = self.sample_conf()
                solution_conf = self.solve(tool_pose, seed_conf=seed_conf, **kwargs)
            if include_failures or (solution_conf is not None):
                yield solution_conf
    def __str__(self):
        return '{}(body={}, tool={}, base={}, joints={})'.format(
            self.__class__.__name__, self.robot, self.tool_name, self.base_name, list(self.joint_names))
