import argparse

import pybullet as p

from pr2_utils import create_inverse_reachability, set_arm_conf, get_other_arm, arm_conf, REST_LEFT_ARM, get_carry_conf
from pybullet_utils import create_box,  disconnect, add_data_path, connect

def main():
    #parser = argparse.ArgumentParser()  # Automatically includes help
    #parser.add_argument('-viewer', action='store_true', help='enable viewer.', )
    #args = parser.parse_args()

    arm = 'right'
    other_arm = get_other_arm(arm)
    grasp_type = 'top'

    client = connect(use_gui=False)
    add_data_path()

    robot = p.loadURDF("pr2_description/pr2_fixed_torso.urdf")
    #plane = p.loadURDF("plane.urdf")
    table = p.loadURDF("table/table.urdf", 0, 0, 0, 0, 0, 0.707107, 0.707107)
    box = create_box(.07, .05, .15)

    print set_arm_conf(robot, arm, get_carry_conf(arm, grasp_type))
    print set_arm_conf(robot, other_arm, arm_conf(other_arm, REST_LEFT_ARM))

    create_inverse_reachability(robot, box, table, arm=arm, grasp_type=grasp_type)
    disconnect()

if __name__ == '__main__':
    main()