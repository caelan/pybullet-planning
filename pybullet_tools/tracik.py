import math
import os.path

import numpy as np
from tracikpy import TracIKSolver

from pybullet_tools.utils import Pose, multiply, invert, tform_from_pose, get_model_info, BASE_LINK, \
    get_link_name, link_from_name, get_joint_name, joint_from_name, parent_link_from_joint, joints_from_names, \
    links_from_names, get_link_pose, draw_pose, set_joint_positions, get_joint_positions, get_joint_limits, \
    CIRCULAR_LIMITS, get_custom_limits


class IKSolver(object):
    def __init__(self, body, tool_link, first_joint=None, tool_offset=Pose(), custom_limits={},
                 seed=None, max_time=5e-3, error=1e-5): #, **kwargs):
        self.tool_link = link_from_name(body, tool_link)
        if first_joint is None:
            self.base_link = BASE_LINK
        else:
            first_joint = joint_from_name(body, first_joint)
            self.base_link = parent_link_from_joint(body, first_joint)
        # joints = get_joint_ancestors(body, self.tool_link)[1:] # get_link_ancestors
        # movable_joints = prune_fixed_joints(body, joints)
        # print([get_joint_name(body, joint) for joint in movable_joints])

        self.body = body
        urdf_info = get_model_info(body)
        self.urdf_path = os.path.abspath(urdf_info.path)
        self.ik_solver = TracIKSolver(
            urdf_file=self.urdf_path,
            base_link=get_link_name(self.body, self.base_link),
            tip_link=get_link_name(self.body, self.tool_link),
            timeout=max_time, epsilon=error,
            solve_type='Speed', # Speed | Distance | Manipulation1 | Manipulation2
        )
        self.ik_solver.joint_limits = list(get_custom_limits(
            self.body, self.joints, custom_limits=custom_limits, circular_limits=CIRCULAR_LIMITS))

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
    def reference_conf(self):
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

    def get_conf(self):
        return get_joint_positions(self.body, self.joints)
    def set_conf(self, conf):
        assert conf is not None
        set_joint_positions(self.body, self.joints, conf)
    def sample_conf(self):
        return self.random_generator.uniform(*self.joint_limits)

    def solve(self, tool_pose, seed_conf=None, pos_tolerance=1e-5, ori_tolerance=math.radians(5e-2)):
        pose = self.base_from_world(tool_pose)
        tform = tform_from_pose(pose)
        #if seed_conf is None: # TODO: will use np.random.default_rng()
        #    seed_conf = self.reference_conf
        bx, by, bz = pos_tolerance * np.ones(3)
        brx, bry, brz = ori_tolerance * np.ones(3)
        conf = self.ik_solver.ik(tform, qinit=seed_conf, bx=bx, by=by, bz=bz, brx=brx, bry=bry, brz=brz)
        self.solutions.append((pose, conf))
        return conf
    def solve_current(self, tool_pose, **kwargs): # solve_closest
        return self.solve(tool_pose, seed_conf=self.get_conf(), **kwargs)
    def solve_randomized(self, tool_pose, **kwargs):
        return self.solve(tool_pose, seed_conf=self.sample_conf(), **kwargs)
    def solve_reference(self, tool_pose, **kwargs):
        return self.solve(tool_pose, seed_conf=self.reference_conf, **kwargs)
    def solve_warm(self, tool_pose, **kwargs):
        return self.solve(tool_pose, seed_conf=self.last_solution, **kwargs)
    def generate(self, tool_pose, include_failures=False, **kwargs):
        #start_time = time.time()
        while True:
            #print(elapsed_time(start_time))
            seed_conf = self.sample_conf()
            solution_conf = self.solve(tool_pose, seed_conf=seed_conf, **kwargs)
            if include_failures or (solution_conf is not None):
                yield solution_conf
    def __str__(self):
        return '{}(body={}, tool={}, base={}, joints={})'.format(
            self.__class__.__name__, self.robot, self.tool_name, self.base_name, list(self.joint_names))
