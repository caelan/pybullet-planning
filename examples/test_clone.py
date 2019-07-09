#!/usr/bin/env python

from __future__ import print_function

from pybullet_tools.pr2_utils import PR2_GROUPS
from pybullet_tools.utils import HideOutput, disconnect, set_base_values, joint_from_name, connect, user_input, \
    dump_world, get_link_name, wait_for_user, clone_body, get_link_parent, get_link_descendants, load_model


def test_clone_robot(pr2):
    # TODO: j toggles frames, p prints timings, w is wire, a is boxes
    new_pr2 = clone_body(pr2, visual=True, collision=False)
    #new_pr2 = clone_body_editor(pr2, visual=True, collision=True)
    dump_world()
    #print(load_srdf_collisions())
    #print(load_dae_collisions())

    # TODO: some unimportant quats are off for both URDF and other
    # TODO: maybe all the frames are actually correct when I load things this way?
    # TODO: the visual geometries are off but not the collision frames?
    """
    import numpy as np
    for link in get_links(pr2):
        if not get_visual_data(new_pr2, link): # pybullet.error: Error receiving visual shape info?
            continue
        #print(get_link_name(pr2, link))
        data1 = VisualShapeData(*get_visual_data(pr2, link)[0])
        data2 = VisualShapeData(*get_visual_data(new_pr2, link)[0])
        pose1 = (data1.localVisualFrame_position, data1.localVisualFrame_orientation)
        pose2 = (data2.localVisualFrame_position, data2.localVisualFrame_orientation)

        #pose1 = get_link_pose(pr2, link) # Links are fine
        #pose2 = get_link_pose(new_pr2, link)
        #pose1 = get_joint_parent_frame(pr2, link)
        #pose2 = get_joint_parent_frame(new_pr2, link)
        #pose1 = get_joint_inertial_pose(pr2, link) # Inertia is fine
        #pose2 = get_joint_inertial_pose(new_pr2, link)
        if not np.allclose(pose1[0], pose2[0], rtol=0, atol=1e-3):
            print('Point', get_link_name(pr2, link), link, pose1[0], pose2[0])
            #print(data1)
            #print(data2)
            #print(get_joint_parent_frame(pr2, link), get_joint_parent_frame(new_pr2, link))
            print(get_joint_inertial_pose(pr2, link)) #, get_joint_inertial_pose(new_pr2, link))
            print()
        if not np.allclose(euler_from_quat(pose1[1]), euler_from_quat(pose2[1]), rtol=0, atol=1e-3):
            print('Quat', get_link_name(pr2, link), link, euler_from_quat(pose1[1]), euler_from_quat(pose2[1]))
        joint_info1 = get_joint_info(pr2, link)
        joint_info2 = get_joint_info(new_pr2, link)
        # TODO: the axis is off for some of these
        if not np.allclose(joint_info1.jointAxis, joint_info2.jointAxis, rtol=0, atol=1e-3):
            print('Axis', get_link_name(pr2, link), link, joint_info1.jointAxis, joint_info2.jointAxis)
    """
    set_base_values(new_pr2, (2, 0, 0))
    wait_for_user()
    # TODO: the drake one has a large out-of-place cylinder as well

def test_clone_arm(pr2):
    first_joint_name = PR2_GROUPS['left_arm'][0]
    first_joint = joint_from_name(pr2, first_joint_name)
    parent_joint = get_link_parent(pr2, first_joint)
    print(get_link_name(pr2, parent_joint), parent_joint, first_joint_name, first_joint)
    print(get_link_descendants(pr2, first_joint)) # TODO: least common ancestor
    links = [first_joint] + get_link_descendants(pr2, first_joint)
    new_arm = clone_body(pr2, links=links, collision=False)
    dump_world()
    set_base_values(pr2, (-2, 0, 0))
    wait_for_user()

def main():
    connect(use_gui=True)

    with HideOutput():
        pr2 = load_model("models/pr2_description/pr2.urdf")
    test_clone_robot(pr2)
    test_clone_arm(pr2)

    user_input('Finish?')
    disconnect()


if __name__ == '__main__':
    main()