import pybullet as p
import time
import argparse
import numpy as np

from pybullet_utils import set_base_values, joint_from_name, set_joint_position, \
    set_joint_positions, add_data_path, connect, plan_base_motion, plan_joint_motion, control_joints, \
    enable_gravity, get_joint_positions
from pr2_utils import TOP_HOLDING_LEFT_ARM, LEFT_ARM_LINK, LEFT_JOINT_NAMES, RIGHT_JOINT_NAMES, TOOL_POSE, TORSO_JOINT, \
    TOP_HOLDING_RIGHT_ARM, get_top_grasps, REST_RIGHT_ARM, SIDE_HOLDING_LEFT_ARM

def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    connect(use_gui=args.viewer)
    add_data_path()

    plane = p.loadURDF("plane.urdf")
    table = p.loadURDF("table/table.urdf", 0, 0, 0, 0, 0, 0.707107, 0.707107)
    pr2 = p.loadURDF("pr2_description/pr2.urdf", useFixedBase=True)

    base_start = (-2, -2, 0)
    base_goal = (2, 2, 0)
    base_goal = base_start
    p.addUserDebugLine(base_start, base_goal, lineColorRGB=(1, 1, 0)) # addUserDebugText
    print base_start, base_goal

    arm_start = SIDE_HOLDING_LEFT_ARM
    arm_goal = TOP_HOLDING_LEFT_ARM

    set_base_values(pr2, base_start)
    left_joints = [joint_from_name(pr2, name) for name in LEFT_JOINT_NAMES]
    right_joints = [joint_from_name(pr2, name) for name in RIGHT_JOINT_NAMES]
    set_joint_positions(pr2, left_joints, arm_start)
    set_joint_positions(pr2, right_joints, REST_RIGHT_ARM)
    set_joint_position(pr2, joint_from_name(pr2, TORSO_JOINT), 0.2)

    raw_input('Plan Base?')
    base_path = plan_base_motion(pr2, base_goal)
    if base_path is None:
        return
    print len(base_path)
    for bq in base_path:
        set_base_values(pr2, bq)
        #raw_input('Continue?')
        time.sleep(0.05)

    raw_input('Plan Arm?')
    arm_path = plan_joint_motion(pr2, left_joints, arm_goal)
    if base_path is None:
        return
    print len(arm_path)
    for q in arm_path:
        set_joint_positions(pr2, left_joints, q)
        #raw_input('Continue?')
        time.sleep(0.01)

    raw_input('Control Arm?')
    real_time = False
    enable_gravity()
    p.setRealTimeSimulation(real_time)
    while not np.allclose(get_joint_positions(pr2, left_joints), arm_start, atol=1e-3):
        control_joints(pr2, left_joints, arm_start)
        if not real_time:
            p.stepSimulation()
            #time.sleep(0.01)

    raw_input('Finish?')
    p.disconnect()

if __name__ == '__main__':
    main()