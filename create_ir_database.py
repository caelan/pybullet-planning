import argparse

import pybullet as p

from pr2_utils import TOP_HOLDING_LEFT_ARM, LEFT_ARM_LINK, LEFT_JOINT_NAMES, RIGHT_JOINT_NAMES, TORSO_JOINT, \
    REST_RIGHT_ARM, \
    load_inverse_reachability, create_inverse_reachability
from pybullet_utils import get_joint_type, is_movable, get_joint_limits, create_box, get_max_velocity, get_num_joints, \
    get_movable_joints, get_joint_name, get_body_name, get_link_pose, joint_from_name, link_from_name, set_joint_position, \
    get_joint_position, disconnect, \
    get_body_names, get_joint_names, get_colliding_links, self_collision, set_joint_positions, get_joint_positions, \
    add_data_path, connect

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


    torso = joint_from_name(pr2, TORSO_JOINT)
    torso_point, torso_quat = get_link_pose(pr2, torso)

    create_inverse_reachability(pr2, box, table)
    ir_database = load_inverse_reachability()
    print len(ir_database)
    disconnect()


if __name__ == '__main__':
    main()