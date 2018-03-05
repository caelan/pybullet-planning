import pybullet as p
import time
import pybullet_data
import argparse
import numpy as np

REST_LEFT_ARM = [2.13539289, 1.29629967, 3.74999698, -0.15000005, 10000., -0.10000004, 10000.]

LEFT_ARM_LINK = 'l_gripper_palm_link'

LEFT_JOINT_NAMES = ['l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint']
RIGHT_JOINT_NAMES = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
HEAD_JOINT_NAMES = ['head_pan_joint', 'head_tilt_joint']

LEFT_GRIPPER_NAME = 'l_gripper_l_finger_joint'

def rightarm_from_leftarm(config):
  right_from_left = np.array([-1, 1, -1, 1, -1, 1, 1])
  return config*right_from_left

REST_RIGHT_ARM = rightarm_from_leftarm(REST_LEFT_ARM)

LEFT_ARM_JOINTS = [15,16,17,18,19,20,21]
RIGHT_ARM_JOINTS = [27,28,29,30,31,32,33]
HEAD_JOINTS = [13,14]
# openrave-robot.py robots/pr2-beta-static.zae --info manipulators

TORSO_JOINT = 'torso_lift_joint'

#REVOLUTE_LIMITS = -np.pi, np.pi
REVOLUTE_LIMITS = -10000, 10000

BASE_LIMITS = ([-2.5, -2.5, 0], [2.5, 2.5, 0])

class Pose(object):
    def __init__(self, position, orientation):
        self.position = position
        self.orientation = orientation

class Conf(object):
    def __init__(self):
        pass

# https://docs.google.com/document/d/10sXEhzFRSnvFcl3XxNGhnD4N2SedqwdAvK3dsihxVUA/edit#

def get_joint_type(body, joint):
    return p.getJointInfo(body, joint)[2]

def get_joint_limits(body, joint):
    lower = p.getJointInfo(body, joint)[8]
    upper = p.getJointInfo(body, joint)[9]
    if upper < lower:
        return REVOLUTE_LIMITS
    return lower, upper

def get_num_joints(body):
    return p.getNumJoints(body)

def get_joint_name(body, joint):
    return p.getJointInfo(body, joint)[1]

def get_link_name(body, link):
    return p.getJointInfo(body, link)[11]

def get_name(body):
    return p.getBodyInfo(body)[1]

def get_base_link(body):
    return p.getBodyInfo(body)[0]

def get_pose(body):
    point, quat = p.getBasePositionAndOrientation(body) # [x,y,z,w]
    return np.concatenate([point, quat])

def set_pose(body, point, quat):
    p.resetBasePositionAndOrientation(body, point, quat)

def set_point(body, point):
    _, quat = p.getBasePositionAndOrientation(body)
    p.resetBasePositionAndOrientation(body, point, quat)

def get_link_pose(body, link):
    point, quat = p.getLinkState(body, link)
    return np.concatenate([point, quat])

def joint_from_name(body, name):
    for joint in xrange(get_num_joints(body)):
        if get_joint_name(body, joint) == name:
            return joint
    raise ValueError(body, name)

def body_from_name(name):
    for body in xrange(get_num_bodies()):
        if get_name(body) == name:
            return body
    raise ValueError(name)

#def set_joint(body, joint, value):
#    p.setJointMotorControl2(bodyUniqueId=body,
#                            jointIndex=joint,
#                            controlMode=p.POSITION_CONTROL,
#                            targetPosition=value,
#                            force=maxForce)

def set_joint_position(body, joint, value):
    p.resetJointState(body, joint, value)

def get_joint_position(body, joint):
    return p.getJointState(body, joint)[0]

def get_num_bodies():
    return p.getNumBodies()

def get_bodies():
    return range(get_num_bodies())

def get_body_names():
    return map(get_name, get_bodies())

def get_joints(body):
    return range(get_num_joints(body))

def get_joint_names(body):
    return [get_joint_name(body, joint) for joint in get_joints(body)]

def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler)

def euler_from_quat(quat):
    return p.getEulerFromQuaternion(quat)

def z_rotation(theta):
    return quat_from_euler([0, 0, theta])

def matrix_from_quat(quat):
    return p.getMatrixFromQuaternion(quat)

def quat_from_matrix(matrix):
    return p.getMatrixFromQuaternion(matrix)

def pairwise_collision(body1, body2, max_distance=0.001): # 10000
    return len(p.getClosestPoints(body1, body2, max_distance)) != 0 # getContactPoints

def env_collision(body1):
    for body2 in get_bodies():
        if (body1 != body2) and pairwise_collision(body1, body2):
            print body1, body2
            return True
    return False

def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    client = p.connect(p.GUI) if args.viewer else p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
    print pybullet_data.getDataPath()
    p.setGravity(0, 0, -10)
    planeId = p.loadURDF("plane.urdf")
    table = p.loadURDF("table/table.urdf", 1.000000, -0.200000, 0.000000, 0.000000, 0.000000, 0.707107, 0.707107)

    cubeStartPos = [0, 0, 1]
    cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    # boxId = p.loadURDF("r2d2.urdf",cubeStartPos, cubeStartOrientation)
    # boxId = p.loadURDF("pr2.urdf")
    pr2 = p.loadURDF("/Users/caelan/Programs/Installation/pr2_description/pr2_local.urdf", useFixedBase=False)
    #pr2 = p.loadURDF("pr2_description/urdf/pr2_simplified.urdf", useFixedBase=False)

    print pr2
    # for i in range (10000):
    #    p.stepSimulation()
    #    time.sleep(1./240.)

    print get_joint_names(pr2)
    print joint_from_name(pr2, TORSO_JOINT)
    print get_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT))

    raw_input('Continue?')

    print set_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT), 0.2)  # Updates automatically

    cubePos, cubeOrn = p.getBasePositionAndOrientation(pr2)

    print get_name(pr2)
    print get_body_names()
    # print p.getBodyUniqueId(pr2)
    print get_joint_names(pr2)

    #for joint, value in zip(LEFT_ARM_JOINTS, REST_LEFT_ARM):
    #    set_joint_position(pr2, joint, value)
    for name, value in zip(LEFT_JOINT_NAMES, REST_LEFT_ARM):
        joint = joint_from_name(pr2, name)
        #print name, joint, get_joint_position(pr2, joint), value
        print name, get_joint_limits(pr2, joint), get_joint_type(pr2, joint), get_link_name(pr2, joint)
        set_joint_position(pr2, joint, value)
        #print name, joint, get_joint_position(pr2, joint), value
    for name, value in zip(RIGHT_JOINT_NAMES, REST_RIGHT_ARM):
        set_joint_position(pr2, joint_from_name(pr2, name), value)

    print p.getNumJoints(pr2)
    jointId = 0
    print p.getJointInfo(pr2, jointId)
    print p.getJointState(pr2, jointId)

    print(cubePos, cubeOrn)

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

    # TODO: self collisions
    for i in xrange(10):
        for name in LEFT_JOINT_NAMES:
            joint = joint_from_name(pr2, name)
            value = np.random.uniform(*get_joint_limits(pr2, joint))
            set_joint_position(pr2, joint, value)
        raw_input('Continue?')


    raw_input('Finish?')
    p.disconnect()

    # createConstraint


if __name__ == '__main__':
    main()