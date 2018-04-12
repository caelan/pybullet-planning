import pybullet as p
import time

from pr2_utils import inverse_kinematics_helper, get_top_grasps
from utils import get_pose, set_pose, get_link_pose, body_from_end_effector, get_movable_joints, get_configuration, \
    set_joint_positions, get_constraints, grasp_constraint, enable_real_time, disable_real_time, joint_controller, \
    enable_gravity, get_refine_fn, input, wait_for_duration, link_from_name, get_body_name, sample_placement, \
    end_effector_from_body, approach_from_grasp, plan_joint_motion, GraspInfo, Pose, INF, Point

GRASP_INFO = {
    'top': GraspInfo(lambda body: get_top_grasps(body, under=True, tool_pose=Pose(),
                                                 max_width=INF,  grasp_length=0),
                     Pose(0.1*Point(z=1))),
}
TOOL_FRAMES = {
    'iiwa14': 'iiwa_link_ee_kuka', # iiwa_link_ee
}

class BodyPose(object):
    def __init__(self, body, pose=None):
        if pose is None:
            pose = get_pose(body)
        self.body = body
        self.pose = pose
    def assign(self):
        set_pose(self.body, self.pose)
    def __repr__(self):
        return 'p{}'.format(id(self) % 1000)


class BodyGrasp(object):
    def __init__(self, body, grasp_pose, approach_pose, robot, link):
        self.body = body
        self.grasp_pose = grasp_pose
        self.approach_pose = approach_pose
        self.robot = robot
        self.link = link
    #def constraint(self):
    #    grasp_constraint()
    def assign(self):
        end_effector_pose = get_link_pose(self.robot, self.link)
        body_pose = body_from_end_effector(end_effector_pose, self.grasp_pose)
        set_pose(self.body, body_pose)
    def __repr__(self):
        return 'g{}'.format(id(self) % 1000)


class BodyConf(object):
    def __init__(self, body, configuration=None, joints=None):
        if joints is None:
            joints = get_movable_joints(body)
        if configuration is None:
            configuration = get_configuration(body)
        self.body = body
        self.joints = joints
        self.configuration = configuration
    def assign(self):
        set_joint_positions(self.body, self.joints, self.configuration)
    def __repr__(self):
        return 'q{}'.format(id(self) % 1000)


class BodyPath(object):
    def __init__(self, body, path, joints=None, grasps=[]):
        if joints is None:
            joints = get_movable_joints(body)
        self.body = body
        self.path = path
        self.joints = joints
        self.grasps = grasps
    def bodies(self):
        return set([self.body] + [grasp.body for grasp in self.grasps])
    def iterator(self):
        for i, configuration in enumerate(self.path):
            set_joint_positions(self.body, self.joints, configuration)
            for grasp in self.grasps:
                grasp.assign()
            yield i
    def control(self, real_time=False, dt=0):
        # TODO: just waypoints
        if not get_constraints():
            for grasp in self.grasps:
                grasp_constraint(grasp.body, grasp.robot, grasp.link)
        if real_time:
            enable_real_time()
        else:
            disable_real_time()
        for values in self.path:
            for _ in joint_controller(self.body, self.joints, values):
                if real_time:
                    enable_gravity()
                else:
                    p.stepSimulation()
                time.sleep(dt)
        # TODO: could simulate step

    # def full_path(self, q0=None):
    #     # TODO: could produce savers
    #     if q0 is None:
    #         q0 = Conf(self.tree)
    #     new_path = []
    #     for values in self.sequence:
    #         q = q0.copy()
    #         q[self.positions] = values
    #         if self.holding:  # TODO: cache this
    #             kin_cache = self.tree.doKinematics(q)
    #             for body_id, grasp_pose, model_id in self.holding:
    #                 body_pose = get_world_pose(self.tree, kin_cache, body_id)
    #                 model_pose = object_from_gripper(body_pose, grasp_pose)
    #                 set_pose(self.tree, q, model_id, model_pose)
    #         new_path.append(q)
    #     return new_path
    def refine(self, num_steps=0):
        refine_fn = get_refine_fn(self.body, self.joints, num_steps)
        refined_path = []
        for v1, v2 in zip(self.path, self.path[1:]):
            refined_path += list(refine_fn(v1, v2))
        return self.__class__(self.body, refined_path, self.joints, self.grasps)
    def reverse(self):
        return self.__class__(self.body, self.path[::-1], self.joints, self.grasps)
    def __repr__(self):
        return 't{}'.format(id(self) % 1000)


class Command(object):
    def __init__(self, body_paths):
        self.body_paths = body_paths

    # def full_path(self, q0=None):
    #     if q0 is None:
    #         q0 = Conf(self.tree)
    #     new_path = [q0]
    #     for partial_path in self.body_paths:
    #         new_path += partial_path.full_path(new_path[-1])[1:]
    #     return new_path

    def step(self):
        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():
                msg = '{},{}) step?'.format(i, j)
                input(msg)
                #print(msg)
                #wait_for_interrupt()

    def execute(self, time_step=0.05):
        for i, body_path in enumerate(self.body_paths):
            for j in body_path.iterator():
                #time.sleep(time_step)
                wait_for_duration(time_step)

    def control(self, real_time=False, dt=0): # TODO: real_time
        for body_path in self.body_paths:
            body_path.control(real_time, dt)

    def refine(self, num_steps=0):
        return self.__class__([body_path.refine(num_steps) for body_path in self.body_paths])

    def reverse(self):
        return self.__class__([body_path.reverse() for body_path in reversed(self.body_paths)])

    def __repr__(self):
        return 'c{}'.format(id(self) % 1000)


def get_grasp_gen(robot, grasp_name):
    grasp_info = GRASP_INFO[grasp_name]
    end_effector_link = link_from_name(robot, TOOL_FRAMES[get_body_name(robot)])
    def gen(body):
        grasp_poses = grasp_info.get_grasps(body)
        for grasp_pose in grasp_poses:
            body_grasp = BodyGrasp(body, grasp_pose, grasp_info.approach_pose,
                                   robot, end_effector_link)
            yield (body_grasp,)
    return gen


def get_stable_gen(fixed=[]):
    def gen(body, surface):
        while True:
            pose = sample_placement(body, surface)
            if pose is None:
                continue
            body_pose = BodyPose(body, pose)
            yield (body_pose,)
            # TODO: check collisions
    return gen


def get_ik_fn(robot, fixed=[], teleport=False):
    model_ids = ([robot] + fixed)
    def fn(model, pose, grasp):
        gripper_pose = end_effector_from_body(pose.pose, grasp.grasp_pose)
        approach_pose = approach_from_grasp(grasp.approach_pose, gripper_pose)
        q_approach = inverse_kinematics_helper(robot, grasp.link, approach_pose)
        if (q_approach is None): # or are_colliding(tree, tree.doKinematics(q_approach), model_ids=model_ids):
            return None
        conf = BodyConf(robot, q_approach)
        q_grasp = inverse_kinematics_helper(robot, grasp.link, gripper_pose)
        if (q_grasp is None): # or are_colliding(tree, tree.doKinematics(q_grasp), model_ids=model_ids):
            return None
        if teleport:
            path = [q_approach, q_grasp]
        else:
            conf.assign()
            path = plan_joint_motion(robot, conf.joints, q_grasp, obstacles=[], direct=True)
            if path is None:
                raw_input('Approach motion failed')
                return None
        command = Command([BodyPath(robot, path),
                           BodyPath(robot, path[::-1], grasps=[grasp])])
        return (conf, command)
        # TODO: holding collisions
    return fn


def get_free_motion_gen(robot, fixed=[], teleport=False):
    model_ids = ([robot] + fixed)
    def fn(conf1, conf2):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            conf1.assign()
            path = plan_joint_motion(robot, conf2.joints, conf2.configuration, obstacles=[])
            if path is None:
                raw_input('Free motion failed')
                return None
        command = Command([BodyPath(robot, path)])
        return (command,)
    return fn


def get_holding_motion_gen(robot, fixed=[], teleport=False):
    model_ids = ([robot] + fixed)
    def fn(conf1, conf2, body, grasp):
        assert ((conf1.body == conf2.body) and (conf1.joints == conf2.joints))
        if teleport:
            path = [conf1.configuration, conf2.configuration]
        else:
            # TODO: holding
            conf1.assign()
            path = plan_joint_motion(robot, conf2.joints, conf2.configuration, obstacles=[])
            if path is None:
                raw_input('Holding motion failed')
                return None
        command = Command([BodyPath(robot, path, grasps=[grasp])])
        return (command,)
    return fn


def get_movable_collision_test():
    def test(command, model, pose):
        for partial_path in command.partial_paths:
            if any(info.model_id == model for info in partial_path.holding):
                continue  # Cannot collide with itself
            # TODO: cache the KinematicsCaches
            q = Conf(tree)
            pose.assign(q)  # TODO: compute kinematics trees just for pairs/triplets of objects
            model_ids = partial_path.model_ids() + [model]
            for q in command.full_path(q):
                if are_colliding(tree, tree.doKinematics(q), model_ids=model_ids):
                    raw_input('Movable collision')
                    return True
        return False
    return test
