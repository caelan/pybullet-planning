from __future__ import print_function

import argparse
import math
import numpy as np
import pybullet as p

from pr2_utils import TOP_HOLDING_LEFT_ARM, LEFT_ARM_LINK, LEFT_JOINT_NAMES, RIGHT_JOINT_NAMES, TORSO_JOINT_NAME, \
    REST_RIGHT_ARM, \
    load_inverse_reachability
from create_ir_database import create_inverse_reachability
from utils import get_joint_type, is_movable, get_joint_limits, create_box, get_max_velocity, get_num_joints, \
    get_movable_joints, get_joint_name, get_body_name, get_link_pose, joint_from_name, link_from_name, set_joint_position, \
    get_joint_position, \
    get_body_names, get_joint_names, get_colliding_links, self_collision, set_joint_positions, get_joint_positions, \
    add_data_path, connect, wait_if_gui


#REST_LEFT_ARM = [2.13539289, 1.29629967, 3.74999698, -0.15000005, 10000., -0.10000004, 10000.]

#LEFT_ARM_JOINTS = [15,16,17,18,19,20,21]
#RIGHT_ARM_JOINTS = [27,28,29,30,31,32,33]
#HEAD_JOINTS = [13,14]
# openrave-robot.py robots/pr2-beta-static.zae --info manipulators

#REVOLUTE_LIMITS = -10000, 10000

# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#

# http://openrave.org/docs/0.6.6/robots_overview/
# http://openrave.org/docs/latest_stable/collada_robot_extensions/
# http://openrave.programmingvision.com/wiki/index.php/Format:XML
# https://github.com/davetcoleman/pr2_moveit_config/blob/master/config/pr2.srdf
# http://wiki.ros.org/srdf
# http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/urdf_srdf_tutorial.html#srdf
# http://gazebosim.org/tutorials/?tut=ros_urdf
# http://sdformat.org/
# https://github.com/PR2/pr2_common/tree/kinetic-devel/pr2_description
# http://wiki.ros.org/srdf/review

# https://github.com/rdiankov/openrave/blob/ff43549fb6db281c7bc9a85794b88c29d6522ab4/plugins/baserobots/collisionmaprobot.cpp
# https://github.com/rdiankov/openrave/blob/176a25e4b50cad1d7237596823764278920a750a/python/bindings/openravepy_collisionchecker.cpp
# https://github.com/rdiankov/openrave/blob/9bc6b8d459af18c67074da331895c79f7f2f50b0/plugins/oderave/odecollision.h
# https://github.com/rdiankov/openrave/blob/98b6111423dc7e5fe695dcb8654601b8d87b2a82/src/libopenrave/kinbody.cpp
# https://github.com/rdiankov/openrave/blob/98b6111423dc7e5fe695dcb8654601b8d87b2a82/src/libopenrave/kinbody.cpp

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
    print(len(initially_colliding))
    origin = (0, 0, 0)
    print(p.getNumConstraints())


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

    print(pr2)
    # for i in range (10000):
    #    p.stepSimulation()
    #    time.sleep(1./240.)

    #print(get_joint_names(pr2))
    print([get_joint_name(pr2, joint) for joint in get_movable_joints(pr2)])
    print(get_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT_NAME)))
    #open_gripper(pr2, joint_from_name(pr2, LEFT_GRIPPER))
    #print(get_joint_limits(pr2, joint_from_name(pr2, LEFT_GRIPPER)))
    #print(get_joint_position(pr2, joint_from_name(pr2, LEFT_GRIPPER)))
    print(self_collision(pr2))

    """
    print(p.getNumConstraints())
    constraint = fixed_constraint(pr2, -1, box, -1) # table
    p.changeConstraint(constraint)
    print(p.getNumConstraints())
    print(p.getConstraintInfo(constraint))
    print(p.getConstraintState(constraint))
    p.stepSimulation()
    user_input('Continue?')

    set_point(pr2, (-2, 0, 0))
    p.stepSimulation()
    p.changeConstraint(constraint)
    print(p.getConstraintInfo(constraint))
    print(p.getConstraintState(constraint))
    user_input('Continue?')
    print(get_point(pr2))
    user_input('Continue?')
    """

    # TODO: would be good if we could set the joint directly
    print(set_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT_NAME), 0.2))  # Updates automatically
    print(get_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT_NAME)))
    #return

    left_joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]
    right_joints = [joint_from_name(pr2, name) for name in RIGHT_JOINT_NAMES]
    print(set_joint_positions(pr2, left_joints, TOP_HOLDING_LEFT_ARM)) # TOP_HOLDING_LEFT_ARM | SIDE_HOLDING_LEFT_ARM
    print(set_joint_positions(pr2, right_joints, REST_RIGHT_ARM)) # TOP_HOLDING_RIGHT_ARM | REST_RIGHT_ARM

    print(get_body_name(pr2))
    print(get_body_names())
    # print(p.getBodyUniqueId(pr2))
    print(get_joint_names(pr2))


    #for joint, value in zip(LEFT_ARM_JOINTS, REST_LEFT_ARM):
    #    set_joint_position(pr2, joint, value)
    # for name, value in zip(LEFT_JOINT_NAMES, REST_LEFT_ARM):
    #     joint = joint_from_name(pr2, name)
    #     #print(name, joint, get_joint_position(pr2, joint), value)
    #     print(name, get_joint_limits(pr2, joint), get_joint_type(pr2, joint), get_link_name(pr2, joint))
    #     set_joint_position(pr2, joint, value)
    #     #print(name, joint, get_joint_position(pr2, joint), value)
    # for name, value in zip(RIGHT_JOINT_NAMES, REST_RIGHT_ARM):
    #     set_joint_position(pr2, joint_from_name(pr2, name), value)

    print(p.getNumJoints(pr2))
    jointId = 0
    print(p.getJointInfo(pr2, jointId))
    print(p.getJointState(pr2, jointId))

    # for i in range(10):
    #     #lower, upper = BASE_LIMITS
    #     #q = np.random.rand(len(lower))*(np.array(upper) - np.array(lower)) + lower
    #     q = np.random.uniform(*BASE_LIMITS)
    #     theta = np.random.uniform(*REVOLUTE_LIMITS)
    #     quat = z_rotation(theta)
    #     print(q, theta, quat, env_collision(pr2))
    #     #set_point(pr2, q)
    #     set_pose(pr2, q, quat)
    #     #p.getMouseEvents()
    #     #p.getKeyboardEvents()
    #     user_input('Continue?') # Stalls because waiting for input
    #
    # # TODO: self collisions
    # for i in range(10):
    #     for name in LEFT_JOINT_NAMES:
    #         joint = joint_from_name(pr2, name)
    #         value = np.random.uniform(*get_joint_limits(pr2, joint))
    #         set_joint_position(pr2, joint, value)
    #     user_input('Continue?')



    #start = (-2, -2, 0)
    #set_base_values(pr2, start)
    # #start = get_base_values(pr2)
    # goal = (2, 2, 0)
    # p.addUserDebugLine(start, goal, lineColorRGB=(1, 1, 0)) # addUserDebugText
    # print(start, goal)
    # user_input('Plan?')
    # path = plan_base_motion(pr2, goal)
    # print(path)
    # if path is None:
    #     return
    # print(len(path))
    # for bq in path:
    #     set_base_values(pr2, bq)
    #     user_input('Continue?')



    # left_joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]
    # for joint in left_joints:
    #     print(joint, get_joint_name(pr2, joint), get_joint_limits(pr2, joint), \
    #         is_circular(pr2, joint), get_joint_position(pr2, joint))
    #
    # #goal = np.zeros(len(left_joints))
    # goal = []
    # for name, value in zip(LEFT_JOINT_NAMES, REST_LEFT_ARM):
    #     joint = joint_from_name(pr2, name)
    #     goal.append(wrap_joint(pr2, joint, value))
    #
    # path = plan_joint_motion(pr2, left_joints, goal)
    # print(path)
    # for q in path:s
    #     set_joint_positions(pr2, left_joints, q)
    #     user_input('Continue?')

    print(p.JOINT_REVOLUTE, p.JOINT_PRISMATIC, p.JOINT_FIXED, p.JOINT_POINT2POINT, p.JOINT_GEAR) # 0 1 4 5 6

    movable_joints = get_movable_joints(pr2)
    print(len(movable_joints))
    for joint in range(get_num_joints(pr2)):
        if is_movable(pr2, joint):
            print(joint, get_joint_name(pr2, joint), get_joint_type(pr2, joint), get_joint_limits(pr2, joint))

    #joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]
    #set_joint_positions(pr2, joints, sample_joints(pr2, joints))
    #print(get_joint_positions(pr2, joints)) # Need to print before the display updates?



    # set_base_values(pr2, (1, -1, -np.pi/4))
    # movable_joints = get_movable_joints(pr2)
    # gripper_pose = get_link_pose(pr2, link_from_name(pr2, LEFT_ARM_LINK))
    # print(gripper_pose)
    # print(get_joint_positions(pr2, movable_joints))
    # p.addUserDebugLine(origin, gripper_pose[0], lineColorRGB=(1, 0, 0))
    # p.stepSimulation()
    # user_input('Pre2 IK')
    # set_joint_positions(pr2, left_joints, SIDE_HOLDING_LEFT_ARM) # TOP_HOLDING_LEFT_ARM | SIDE_HOLDING_LEFT_ARM
    # print(get_joint_positions(pr2, movable_joints))
    # p.stepSimulation()
    # user_input('Pre IK')
    # conf = inverse_kinematics(pr2, gripper_pose) # Doesn't automatically set configuraitons
    # print(conf)
    # print(get_joint_positions(pr2, movable_joints))
    # set_joint_positions(pr2, movable_joints, conf)
    # print(get_link_pose(pr2, link_from_name(pr2, LEFT_ARM_LINK)))
    # #print(get_joint_positions(pr2, movable_joints))
    # p.stepSimulation()
    # user_input('Post IK')
    # return

    # print(pose_from_tform(TOOL_TFORM))
    # gripper_pose = get_link_pose(pr2, link_from_name(pr2, LEFT_ARM_LINK))
    # #gripper_pose = multiply(gripper_pose, TOOL_POSE)
    # #gripper_pose = get_gripper_pose(pr2)
    # for i, grasp_pose in enumerate(get_top_grasps(box)):
    #     grasp_pose = multiply(TOOL_POSE, grasp_pose)
    #     box_pose = multiply(gripper_pose, grasp_pose)
    #     set_pose(box, *box_pose)
    #     print(get_pose(box))
    #     user_input('Grasp {}'.format(i))
    # return

    torso = joint_from_name(pr2, TORSO_JOINT_NAME)
    torso_point, torso_quat = get_link_pose(pr2, torso)

    #torso_constraint = p.createConstraint(pr2, torso, -1, -1,
    #                   p.JOINT_FIXED, jointAxis=[0] * 3,  # JOINT_FIXED
    #                   parentFramePosition=torso_point,
    #                   childFramePosition=torso_quat)

    create_inverse_reachability(pr2, box, table)
    ir_database = load_inverse_reachability()
    print(len(ir_database))


    return


    link = link_from_name(pr2, LEFT_ARM_LINK)
    point, quat = get_link_pose(pr2, link)
    print(point, quat)
    p.addUserDebugLine(origin, point, lineColorRGB=(1, 1, 0))  # addUserDebugText
    user_input('Continue?')

    current_conf = get_joint_positions(pr2, movable_joints)

    #ik_conf = p.calculateInverseKinematics(pr2, link, point)
    #ik_conf = p.calculateInverseKinematics(pr2, link, point, quat)

    min_limits = [get_joint_limits(pr2, joint)[0] for joint in movable_joints]
    max_limits = [get_joint_limits(pr2, joint)[1] for joint in movable_joints]
    max_velocities = [get_max_velocity(pr2, joint) for joint in movable_joints] # Range of Jacobian
    print(min_limits)
    print(max_limits)
    print(max_velocities)
    ik_conf = p.calculateInverseKinematics(pr2, link, point, quat, lowerLimits=min_limits,
                                           upperLimits=max_limits, jointRanges=max_velocities, restPoses=current_conf)


    value_from_joint = dict(zip(movable_joints, ik_conf))
    print([value_from_joint[joint] for joint in joints])

    #print(len(ik_conf), ik_conf)
    set_joint_positions(pr2, movable_joints, ik_conf)
    #print(len(movable_joints), get_joint_positions(pr2, movable_joints))
    print(get_joint_positions(pr2, joints))

    user_input('Finish?')

    p.disconnect()

    # createConstraint

def get_contact_links(contact):
    _, body1, body2, link1, link2 = contact[:5]
    distance = contact[8]
    return (body1, link1), (body2, link2), distance

def get_colliding_links(body, max_distance=-0.001):
    contacts = p.getClosestPoints(body, body, max_distance) # -1 is the base
    colliding = set()
    for (_, link1), (_, link2), _ in map(get_contact_links, contacts):
        colliding.update([(link1, link2), (link2, link1)])
    return colliding

def get_safe_colliding_links(body):
    return get_adjacent_links(body) | get_fixed_links(body)

def self_collision(body, max_distance=0):
    # GetNonAdjacentLinks | GetAdjacentLinks
    contacts = p.getClosestPoints(body, body, max_distance) # -1 is the base
    #print(contacts)
    adjacent = get_safe_colliding_links(body)
    #print(fixed)
    #print(sorted(get_adjacent_links(body)))
    colliding_not_adjacent = {(link1, link2, distance) for (_, link1), (_, link2), distance in map(get_contact_links, contacts)
           if (link1 != link2) and ((link1, link2) not in adjacent) and ((link2, link1) not in adjacent)}
    colliding_not_adjacent = list(colliding_not_adjacent)
    #print(colliding_not_adjacent)
    #print([(get_link_name(body, link1), get_link_name(body, link2), distance)
    #       for (link1, link2, distance) in colliding_not_adjacent])
    # TODO: could compute initially colliding links and discount those collisions
    return len(colliding_not_adjacent) != 0

#def filtered_self_collision(body, acceptable=tuple(), max_distance=0):
#    contacts = p.getClosestPoints(body, body, max_distance) # -1 is the base
#    for (_, link1), (_, link2), _ in map(get_contact_links, contacts):
#        if (link1 != link2) and (link1, link2) not in acceptable:
#            return True
#    return False

"""
if data.linkIndex != BASE_LINK:
    # parent_from_joint
    print(data.objectUniqueId, data.linkIndex)
    old_pose = get_joint_parent_frame(data.objectUniqueId, data.linkIndex)  # Incorrect
    new_pose = get_local_link_pose(data.objectUniqueId, data.linkIndex)  # Correct

    # parent1_from_joint * joint_from_visual = parent2_from_joint * X
    # parent2_from_joint^-1 * parent1_from_joint * joint_from_visual
    # parent1_from_parent2 * ?joint_from_visual?
    #pose = multiply(invert(new_pose), old_pose, pose)
    #pose = multiply(invert(old_pose), new_pose, pose)
    #pose = multiply(pose, invert(new_pose), old_pose)
    #pose = multiply(pose, invert(old_pose), new_pose)
    #pose = multiply(pose, new_pose, invert(old_pose))
    pose = Pose(Point(x=5)) # TODO: so this is actually taken into account
    print(pose)
"""

def experimental_inverse_kinematics(robot, link, pose,
                       null_space=False, max_iterations=200, tolerance=1e-3):
    (point, quat) = pose
    # https://github.com/bulletphysics/bullet3/blob/389d7aaa798e5564028ce75091a3eac6a5f76ea8/examples/SharedMemory/PhysicsClientC_API.cpp
    # https://github.com/bulletphysics/bullet3/blob/c1ba04a5809f7831fa2dee684d6747951a5da602/examples/pybullet/examples/inverse_kinematics_husky_kuka.py
    joints = get_joints(robot) # Need to have all joints (although only movable returned)
    movable_joints = get_movable_joints(robot)
    current_conf = get_joint_positions(robot, joints)

    # TODO: sample other values for the arm joints as the reference conf
    min_limits = [get_joint_limits(robot, joint)[0] for joint in joints]
    max_limits = [get_joint_limits(robot, joint)[1] for joint in joints]
    #min_limits = current_conf
    #max_limits = current_conf
    #max_velocities = [get_max_velocity(robot, joint) for joint in joints] # Range of Jacobian
    max_velocities = [10000]*len(joints)
    # TODO: cannot have zero velocities
    # TODO: larger definitely better for velocities
    #damping = tuple(0.1*np.ones(len(joints)))

    #t0 = time.time()
    #kinematic_conf = get_joint_positions(robot, movable_joints)
    for iterations in range(max_iterations): # 0.000863273143768 / iteration
        # TODO: return none if no progress
        if null_space:
            kinematic_conf = p.calculateInverseKinematics(robot, link, point, quat,
                                                          lowerLimits=min_limits, upperLimits=max_limits,
                                                          jointRanges=max_velocities, restPoses=current_conf,
                                                          #jointDamping=damping,
                                                         )
        else:
            kinematic_conf = p.calculateInverseKinematics(robot, link, point, quat)
        if (kinematic_conf is None) or any(map(math.isnan, kinematic_conf)):
            return None
        set_joint_positions(robot, movable_joints, kinematic_conf)
        link_point, link_quat = get_link_pose(robot, link)
        if np.allclose(link_point, point, atol=tolerance) and np.allclose(link_quat, quat, atol=tolerance):
            #print(iterations)
            break
    else:
        return None
    if violates_limits(robot, movable_joints, kinematic_conf):
        return None
    #total_time = (time.time() - t0)
    #print(total_time)
    #print((time.time() - t0)/max_iterations)
    return kinematic_conf

"""
def clone_body_editor(body, collision=True, visual=True):
    #from pybullet_utils.urdfEditor import UrdfEditor
    from urdfEditor import UrdfEditor
    editor = UrdfEditor()
    editor.initializeFromBulletBody(body, physicsClientId=CLIENT)
    return editor.createMultiBody(physicsClientId=CLIENT) # pybullet.error: createVisualShapeArray failed.

    # TODO: failure is because broken mesh files
    #return body_from_editor(editor, collision=collision, visual=visual)
    #filename = 'temp.urdf'
    #editor.saveUrdf(filename)
    #new_body = load_model(filename)
    #os.remove(filename)
    #return new_body
    # TODO: problem with collision is that URDF editor is storing mesh when it should be geom
    # example: fl_caster_l_wheel_link # <mesh filename="unknown_file"/>

def save_body(body, filename):
    #from pybullet_utils.urdfEditor import UrdfEditor
    from urdfEditor import UrdfEditor
    editor = UrdfEditor()
    editor.initializeFromBulletBody(body, physicsClientId=CLIENT)
    editor.saveUrdf(filename)
"""

"""
def body_from_editor(editor, collision=True, visual=True):
    #basePosition = [0, 0, 0]
    #baseOrientation = unit_quat()
    if (len(editor.urdfLinks) == 0):
        return None

    base = editor.urdfLinks[0]  # assume link[0] is base
    baseMass = base.urdf_inertial.mass
    baseCollisionShapeIndex = -1
    baseVisualShapeIndex = -1
    baseShapeTypeArray = []
    baseRadiusArray = []
    baseHalfExtentsArray = []
    lengthsArray = []
    fileNameArray = []
    meshScaleArray = []
    basePositionsArray = []
    baseOrientationsArray = []
    if base.urdf_collision_shapes and collision:
        for v in base.urdf_collision_shapes:
            baseShapeTypeArray.append(v.geom_type)
            baseHalfExtentsArray.append([0.5 * v.geom_extents[0], 0.5 * v.geom_extents[1], 0.5 * v.geom_extents[2]])
            baseRadiusArray.append(v.geom_radius)
            lengthsArray.append(v.geom_length)
            fileNameArray.append(v.geom_meshfilename)
            meshScaleArray.append(v.geom_meshscale)
            basePositionsArray.append(v.origin_xyz)
            baseOrientationsArray.append(p.getQuaternionFromEuler(v.origin_rpy))
        baseCollisionShapeIndex = p.createCollisionShapeArray(shapeTypes=baseShapeTypeArray,
                                                              radii=baseRadiusArray,
                                                              halfExtents=baseHalfExtentsArray,
                                                              lengths=lengthsArray,
                                                              fileNames=fileNameArray,
                                                              meshScales=meshScaleArray,
                                                              collisionFramePositions=basePositionsArray,
                                                              collisionFrameOrientations=baseOrientationsArray)

    if base.urdf_collision_shapes and visual:
        urdfVisuals = base.urdf_visual_shapes
        baseVisualShapeIndex = p.createVisualShapeArray(shapeTypes=[v.geom_type for v in urdfVisuals],
                                                        halfExtents=[[ext * 0.5 for ext in v.geom_extents] for v in
                                                                     urdfVisuals],
                                                        radii=[v.geom_radius for v in urdfVisuals],
                                                        lengths=[v.geom_length[0] for v in urdfVisuals],
                                                        fileNames=[v.geom_meshfilename for v in urdfVisuals],
                                                        meshScales=[v.geom_meshscale for v in urdfVisuals],
                                                        rgbaColors=[v.material_rgba for v in urdfVisuals],
                                                        visualFramePositions=[v.origin_xyz for v in urdfVisuals],
                                                        visualFrameOrientations=[v.origin_rpy for v in urdfVisuals])

    linkMasses = []
    linkCollisionShapeIndices = []
    linkVisualShapeIndices = []
    linkPositions = []
    linkOrientations = []
    linkMeshScaleArray = []
    linkInertialFramePositions = []
    linkInertialFrameOrientations = []
    linkParentIndices = []
    linkJointTypes = []
    linkJointAxis = []
    for joint in editor.urdfJoints:
        link = joint.link
        linkMass = link.urdf_inertial.mass
        linkCollisionShapeIndex = -1
        linkVisualShapeIndex = -1
        linkParentIndex = editor.linkNameToIndex[joint.parent_name]
        linkShapeTypeArray = []
        linkRadiusArray = []
        linkHalfExtentsArray = []
        lengthsArray = []
        fileNameArray = []
        linkPositionsArray = []
        linkOrientationsArray = []

        if link.urdf_collision_shapes and collision:
            for v in link.urdf_collision_shapes:
                linkShapeTypeArray.append(v.geom_type)
                linkHalfExtentsArray.append([0.5 * v.geom_extents[0], 0.5 * v.geom_extents[1], 0.5 * v.geom_extents[2]])
                linkRadiusArray.append(v.geom_radius)
                lengthsArray.append(v.geom_length)
                fileNameArray.append(v.geom_meshfilename)
                linkMeshScaleArray.append(v.geom_meshscale)
                linkPositionsArray.append(v.origin_xyz)
                linkOrientationsArray.append(p.getQuaternionFromEuler(v.origin_rpy))
            linkCollisionShapeIndex = p.createCollisionShapeArray(shapeTypes=linkShapeTypeArray,
                                                                  radii=linkRadiusArray,
                                                                  halfExtents=linkHalfExtentsArray,
                                                                  lengths=lengthsArray,
                                                                  fileNames=fileNameArray,
                                                                  meshScales=linkMeshScaleArray,
                                                                  collisionFramePositions=linkPositionsArray,
                                                                  collisionFrameOrientations=linkOrientationsArray)

        if link.urdf_visual_shapes and visual:
            urdfVisuals = link.urdf_visual_shapes
            # TODO: bug in geom_length where the default is [[10]]
            linkVisualShapeIndex = p.createVisualShapeArray(shapeTypes=[v.geom_type for v in urdfVisuals],
                                                            halfExtents=[[ext * 0.5 for ext in v.geom_extents] for v in
                                                                         urdfVisuals],
                                                            radii=[v.geom_radius for v in urdfVisuals],
                                                            lengths=[v.geom_length[0] for v in urdfVisuals],
                                                            fileNames=[v.geom_meshfilename for v in urdfVisuals],
                                                            meshScales=[v.geom_meshscale for v in urdfVisuals],
                                                            rgbaColors=[v.material_rgba for v in urdfVisuals],
                                                            visualFramePositions=[v.origin_xyz for v in urdfVisuals],
                                                            visualFrameOrientations=[v.origin_rpy for v in
                                                                                     urdfVisuals])

        linkMasses.append(linkMass)
        linkCollisionShapeIndices.append(linkCollisionShapeIndex)
        linkVisualShapeIndices.append(linkVisualShapeIndex)
        linkPositions.append(joint.joint_origin_xyz)
        linkOrientations.append(p.getQuaternionFromEuler(joint.joint_origin_rpy))
        linkInertialFramePositions.append(link.urdf_inertial.origin_xyz)
        linkInertialFrameOrientations.append(p.getQuaternionFromEuler(link.urdf_inertial.origin_rpy))
        linkParentIndices.append(linkParentIndex)
        linkJointTypes.append(joint.joint_type)
        linkJointAxis.append(joint.joint_axis_xyz)

    return p.createMultiBody(baseMass, \
                              baseCollisionShapeIndex=baseCollisionShapeIndex,
                              baseVisualShapeIndex=baseVisualShapeIndex,
                              # basePosition=basePosition,
                              # baseOrientation=baseOrientation,
                              baseInertialFramePosition=base.urdf_inertial.origin_xyz,
                              #baseInertialFrameOrientation=base.urdf_inertial.origin_rpy,
                              baseInertialFrameOrientation=p.getQuaternionFromEuler(base.urdf_inertial.origin_rpy),
                              linkMasses=linkMasses,
                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                              linkVisualShapeIndices=linkVisualShapeIndices,
                              linkPositions=linkPositions,
                              linkOrientations=linkOrientations,
                              linkInertialFramePositions=linkInertialFramePositions,
                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                              linkParentIndices=linkParentIndices,
                              linkJointTypes=linkJointTypes,
                              linkJointAxis=linkJointAxis)
"""


if __name__ == '__main__':
    main()