import pybullet as p
import time
import argparse
import os
import random

from pybullet_utils import get_joint_type, is_movable, get_joint_limits, create_box, invert, multiply, \
    get_max_velocity, get_num_joints, get_movable_joints, get_joint_name, get_name, get_point, get_base_values, \
    set_base_values, set_pose, get_link_pose, joint_from_name, link_from_name, set_joint_position, get_joint_position, \
    get_body_names, get_joint_names, pairwise_collision, get_colliding_links, self_collision, env_collision, \
    set_joint_positions, get_joint_positions, sample_placement, sample_reachable_base, add_data_path, connect, \
    filtered_self_collision, get_safe_colliding_links, get_pose, write_pickle, read_pickle, point_from_pose
from pr2_utils import TOP_HOLDING_LEFT_ARM, LEFT_ARM_LINK, LEFT_JOINT_NAMES, RIGHT_JOINT_NAMES, TOOL_POSE, TORSO_JOINT, \
    TOP_HOLDING_RIGHT_ARM, get_top_grasps, REST_RIGHT_ARM, \
    inverse_kinematics, inverse_kinematics_helper


#REST_LEFT_ARM = [2.13539289, 1.29629967, 3.74999698, -0.15000005, 10000., -0.10000004, 10000.]

#LEFT_ARM_JOINTS = [15,16,17,18,19,20,21]
#RIGHT_ARM_JOINTS = [27,28,29,30,31,32,33]
#HEAD_JOINTS = [13,14]
# openrave-robot.py robots/pr2-beta-static.zae --info manipulators

#REVOLUTE_LIMITS = -10000, 10000

# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#


DATABASES_DIR = 'databases'
IR_FILENAME = '{}_{}_ir.pickle'

def load_inverse_reachability(grasp_type='top', arm='leftarm'):
    filename = IR_FILENAME.format(grasp_type, arm)
    path = os.path.join(DATABASES_DIR, filename)
    return read_pickle(path)['gripper_from_base']

def learned_pose_generator(robot, gripper_pose):
    gripper_from_base_list = load_inverse_reachability()
    random.shuffle(gripper_from_base_list)
    for gripper_from_base in gripper_from_base_list:
        base_pose = multiply(gripper_pose, gripper_from_base)
        set_pose(robot, *base_pose)
        yield base_pose

def uniform_pose_generator(robot, gripper_pose):
    point = point_from_pose(gripper_pose)
    while True:
        base_values = sample_reachable_base(robot, point)
        set_base_values(robot, base_values)
        yield get_pose(robot)


def create_inverse_reachability(pr2, box, table, num_samples=500):
    #initially_colliding = get_colliding_links(pr2) | get_safe_colliding_links(pr2)
    link = link_from_name(pr2, LEFT_ARM_LINK)

    #torso = joint_from_name(pr2, TORSO_JOINT)
    #origin = (0, 0, 0)
    movable_joints = get_movable_joints(pr2)
    default_conf = get_joint_positions(pr2, movable_joints)
    gripper_from_base_list = []
    while len(gripper_from_base_list) < num_samples:
        box_pose = sample_placement(box, table)
        #box_pose = ((0, 0, 1), quat_from_euler(np.zeros(3)))
        set_pose(box, *box_pose)
        for grasp_pose in list(get_top_grasps(box))[:1]:
        #for grasp_pose in get_top_grasps(box):
            gripper_pose = multiply(box_pose, invert(grasp_pose))
            #p.addUserDebugLine(origin, gripper_pose[0], lineColorRGB=(1, 1, 0))
            set_joint_positions(pr2, movable_joints, default_conf)
            #set_pose(pr2, *next(uniform_pose_generator(pr2, gripper_pose)))
            set_pose(pr2, *next(learned_pose_generator(pr2, gripper_pose)))

            if pairwise_collision(pr2, table):
                continue

            #print env_collision(pr2), pairwise_collision(pr2, box), pairwise_collision(pr2, pr2)
            #torso_point, torso_quat = get_link_pose(pr2, torso)
            #print get_link_pose(pr2, torso)
            #p.changeConstraint(torso_constraint, jointChildPivot=torso_point,
            #                   jointChildFrameOrientation=torso_quat, maxForce=1000000)

            conf = inverse_kinematics_helper(pr2, link, gripper_pose)
            if (conf is None) or pairwise_collision(pr2, table):
                continue

            #colliding = set(get_colliding_links(pr2)) - set(initially_colliding)
            #print [(get_joint_name(pr2, j1), get_joint_name(pr2, j2)) for (j1, j2) in colliding]
            #raw_input('awefawef')
            #if filtered_self_collision(pr2, acceptable=initially_colliding):
            #    continue

            gripper_from_base = multiply(invert(get_link_pose(pr2, link)), get_pose(pr2))
            gripper_from_base_list.append(gripper_from_base)

    grasp_type = 'top'
    arm = 'leftarm'
    filename = IR_FILENAME.format(grasp_type, arm)
    path = os.path.join(DATABASES_DIR, filename)
    data = {
        'filename': filename,
        'robot': get_name(pr2),
        'grasp_type': grasp_type,
        'arg': arm,
        'carry_conf': TOP_HOLDING_LEFT_ARM,
        'gripper_link': link,
        'gripper_from_base': gripper_from_base_list,
    }
    #write_pickle(path, data)


def main():
    # TODO: teleporting kuka arm
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    client = connect(use_gui=args.viewer)
    add_data_path()

    #planeId = p.loadURDF("plane.urdf")
    table = p.loadURDF("table/table.urdf", 0, 0, 0, 0, 0, 0.707107, 0.707107)
    box = create_box(.07, .05, .15)


    # boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
    #pr2 = p.loadURDF("pr2_description/pr2.urdf")
    pr2 = p.loadURDF("pr2_description/pr2_fixed_torso.urdf")
    #pr2 = p.loadURDF("/Users/caelan/Programs/Installation/pr2_drake/pr2_local2.urdf",)
                     #useFixedBase=0,)
                     #flags=p.URDF_USE_SELF_COLLISION)
                     #flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)
                     #flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
    #pr2 = p.loadURDF("pr2_drake/urdf/pr2_simplified.urdf", useFixedBase=False)
    initially_colliding = get_colliding_links(pr2)
    print len(initially_colliding)
    origin = (0, 0, 0)
    print p.getNumConstraints()


    # TODO: no way of controlling the base position by itself
    # TODO: PR2 seems to collide with itself frequently
    # real_time = False
    # p.setRealTimeSimulation(real_time)
    # left_joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]
    # control_joints(pr2, left_joints, TOP_HOLDING_LEFT_ARM)
    # while True:
    #     control_joints(pr2, left_joints, TOP_HOLDING_LEFT_ARM)
    #     if not real_time:
    #         p.stepSimulation()

    # A CollisionMap robot allows the user to specify self-collision regions indexed by the values of two joints.

    # GetRigidlyAttachedLinks

    print pr2
    # for i in range (10000):
    #    p.stepSimulation()
    #    time.sleep(1./240.)

    #print get_joint_names(pr2)
    print [get_joint_name(pr2, joint) for joint in get_movable_joints(pr2)]
    print get_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT))
    #open_gripper(pr2, joint_from_name(pr2, LEFT_GRIPPER))
    #print get_joint_limits(pr2, joint_from_name(pr2, LEFT_GRIPPER))
    #print get_joint_position(pr2, joint_from_name(pr2, LEFT_GRIPPER))
    print self_collision(pr2)

    """
    print p.getNumConstraints()
    constraint = fixed_constraint(pr2, -1, box, -1) # table
    p.changeConstraint(constraint)
    print p.getNumConstraints()
    print p.getConstraintInfo(constraint)
    print p.getConstraintState(constraint)
    p.stepSimulation()
    raw_input('Continue?')

    set_point(pr2, (-2, 0, 0))
    p.stepSimulation()
    p.changeConstraint(constraint)
    print p.getConstraintInfo(constraint)
    print p.getConstraintState(constraint)
    raw_input('Continue?')
    print get_point(pr2)
    raw_input('Continue?')
    """

    # TODO: would be good if we could set the joint directly
    print set_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT), 0.2)  # Updates automatically
    print get_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT))
    #return

    left_joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]
    right_joints = [joint_from_name(pr2, name) for name in RIGHT_JOINT_NAMES]
    print set_joint_positions(pr2, left_joints, TOP_HOLDING_LEFT_ARM) # TOP_HOLDING_LEFT_ARM | SIDE_HOLDING_LEFT_ARM
    print set_joint_positions(pr2, right_joints, REST_RIGHT_ARM) # TOP_HOLDING_RIGHT_ARM | REST_RIGHT_ARM

    print get_name(pr2)
    print get_body_names()
    # print p.getBodyUniqueId(pr2)
    print get_joint_names(pr2)


    #for joint, value in zip(LEFT_ARM_JOINTS, REST_LEFT_ARM):
    #    set_joint_position(pr2, joint, value)
    # for name, value in zip(LEFT_JOINT_NAMES, REST_LEFT_ARM):
    #     joint = joint_from_name(pr2, name)
    #     #print name, joint, get_joint_position(pr2, joint), value
    #     print name, get_joint_limits(pr2, joint), get_joint_type(pr2, joint), get_link_name(pr2, joint)
    #     set_joint_position(pr2, joint, value)
    #     #print name, joint, get_joint_position(pr2, joint), value
    # for name, value in zip(RIGHT_JOINT_NAMES, REST_RIGHT_ARM):
    #     set_joint_position(pr2, joint_from_name(pr2, name), value)

    print p.getNumJoints(pr2)
    jointId = 0
    print p.getJointInfo(pr2, jointId)
    print p.getJointState(pr2, jointId)

    # for i in xrange(10):
    #     #lower, upper = BASE_LIMITS
    #     #q = np.random.rand(len(lower))*(np.array(upper) - np.array(lower)) + lower
    #     q = np.random.uniform(*BASE_LIMITS)
    #     theta = np.random.uniform(*REVOLUTE_LIMITS)
    #     quat = z_rotation(theta)
    #     print q, theta, quat, env_collision(pr2)
    #     #set_point(pr2, q)
    #     set_pose(pr2, q, quat)
    #     #p.getMouseEvents()
    #     #p.getKeyboardEvents()
    #     raw_input('Continue?') # Stalls because waiting for input
    #
    # # TODO: self collisions
    # for i in xrange(10):
    #     for name in LEFT_JOINT_NAMES:
    #         joint = joint_from_name(pr2, name)
    #         value = np.random.uniform(*get_joint_limits(pr2, joint))
    #         set_joint_position(pr2, joint, value)
    #     raw_input('Continue?')



    #start = (-2, -2, 0)
    #set_base_values(pr2, start)
    # #start = get_base_values(pr2)
    # goal = (2, 2, 0)
    # p.addUserDebugLine(start, goal, lineColorRGB=(1, 1, 0)) # addUserDebugText
    # print start, goal
    # raw_input('Plan?')
    # path = plan_base_motion(pr2, goal)
    # print path
    # if path is None:
    #     return
    # print len(path)
    # for bq in path:
    #     set_base_values(pr2, bq)
    #     raw_input('Continue?')



    # left_joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]
    # for joint in left_joints:
    #     print joint, get_joint_name(pr2, joint), get_joint_limits(pr2, joint), \
    #         is_circular(pr2, joint), get_joint_position(pr2, joint)
    #
    # #goal = np.zeros(len(left_joints))
    # goal = []
    # for name, value in zip(LEFT_JOINT_NAMES, REST_LEFT_ARM):
    #     joint = joint_from_name(pr2, name)
    #     goal.append(wrap_joint(pr2, joint, value))
    #
    # path = plan_joint_motion(pr2, left_joints, goal)
    # print path
    # for q in path:s
    #     set_joint_positions(pr2, left_joints, q)
    #     raw_input('Continue?')

    print p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, p.JOINT_FIXED, p.JOINT_POINT2POINT, p.JOINT_GEAR # 0 1 4 5 6

    movable_joints = get_movable_joints(pr2)
    print len(movable_joints)
    for joint in xrange(get_num_joints(pr2)):
        if is_movable(pr2, joint):
            print joint, get_joint_name(pr2, joint), get_joint_type(pr2, joint), get_joint_limits(pr2, joint)

    #joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]
    #set_joint_positions(pr2, joints, sample_joints(pr2, joints))
    #print get_joint_positions(pr2, joints) # Need to print before the display updates?



    # set_base_values(pr2, (1, -1, -np.pi/4))
    # movable_joints = get_movable_joints(pr2)
    # gripper_pose = get_link_pose(pr2, link_from_name(pr2, LEFT_ARM_LINK))
    # print gripper_pose
    # print get_joint_positions(pr2, movable_joints)
    # p.addUserDebugLine(origin, gripper_pose[0], lineColorRGB=(1, 0, 0))
    # p.stepSimulation()
    # raw_input('Pre2 IK')
    # set_joint_positions(pr2, left_joints, SIDE_HOLDING_LEFT_ARM) # TOP_HOLDING_LEFT_ARM | SIDE_HOLDING_LEFT_ARM
    # print get_joint_positions(pr2, movable_joints)
    # p.stepSimulation()
    # raw_input('Pre IK')
    # conf = inverse_kinematics(pr2, gripper_pose) # Doesn't automatically set configuraitons
    # print conf
    # print get_joint_positions(pr2, movable_joints)
    # set_joint_positions(pr2, movable_joints, conf)
    # print get_link_pose(pr2, link_from_name(pr2, LEFT_ARM_LINK))
    # #print get_joint_positions(pr2, movable_joints)
    # p.stepSimulation()
    # raw_input('Post IK')
    # return

    # print pose_from_tform(TOOL_TFORM)
    # gripper_pose = get_link_pose(pr2, link_from_name(pr2, LEFT_ARM_LINK))
    # #gripper_pose = multiply(gripper_pose, TOOL_POSE)
    # #gripper_pose = get_gripper_pose(pr2)
    # for i, grasp_pose in enumerate(get_top_grasps(box)):
    #     grasp_pose = multiply(TOOL_POSE, grasp_pose)
    #     box_pose = multiply(gripper_pose, grasp_pose)
    #     set_pose(box, *box_pose)
    #     print get_pose(box)
    #     raw_input('Grasp {}'.format(i))
    # return

    torso = joint_from_name(pr2, TORSO_JOINT)
    torso_point, torso_quat = get_link_pose(pr2, torso)

    #torso_constraint = p.createConstraint(pr2, torso, -1, -1,
    #                   p.JOINT_FIXED, jointAxis=[0] * 3,  # JOINT_FIXED
    #                   parentFramePosition=torso_point,
    #                   childFramePosition=torso_quat)

    create_inverse_reachability(pr2, box, table)
    ir_database = load_inverse_reachability()
    print len(ir_database)


    return


    link = link_from_name(pr2, LEFT_ARM_LINK)
    point, quat = get_link_pose(pr2, link)
    print point, quat
    p.addUserDebugLine(origin, point, lineColorRGB=(1, 1, 0))  # addUserDebugText
    raw_input('Continue?')

    current_conf = get_joint_positions(pr2, movable_joints)

    #ik_conf = p.calculateInverseKinematics(pr2, link, point)
    #ik_conf = p.calculateInverseKinematics(pr2, link, point, quat)

    min_limits = [get_joint_limits(pr2, joint)[0] for joint in movable_joints]
    max_limits = [get_joint_limits(pr2, joint)[1] for joint in movable_joints]
    max_velocities = [get_max_velocity(pr2, joint) for joint in movable_joints] # Range of Jacobian
    print min_limits
    print max_limits
    print max_velocities
    ik_conf = p.calculateInverseKinematics(pr2, link, point, quat, lowerLimits=min_limits,
                                           upperLimits=max_limits, jointRanges=max_velocities, restPoses=current_conf)


    value_from_joint = dict(zip(movable_joints, ik_conf))
    print [value_from_joint[joint] for joint in joints]

    #print len(ik_conf), ik_conf
    set_joint_positions(pr2, movable_joints, ik_conf)
    #print len(movable_joints), get_joint_positions(pr2, movable_joints)
    print get_joint_positions(pr2, joints)

    raw_input('Finish?')

    p.disconnect()

    # createConstraint


if __name__ == '__main__':
    main()