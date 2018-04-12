#!/usr/bin/env python

from __future__ import print_function

from kuka_primitives import BodyPose, BodyConf, Command, get_grasp_gen, get_ik_fn, get_free_motion_gen, \
    get_holding_motion_gen
from utils import WorldSaver, enable_gravity, connect, dump_world, set_pose, Pose, Point, set_default_camera, stable_z, \
    BLOCK_URDF, load_model, wait_for_interrupt, disconnect, DRAKE_IIWA_URDF, input, update_state, disable_real_time

def main():
    connect(use_gui=True)
    disable_real_time()
    robot = load_model(DRAKE_IIWA_URDF)
    floor = load_model('models/short_floor.urdf')
    block = load_model(BLOCK_URDF, fixed_base=False)
    set_pose(block, Pose(Point(y=0.5, z=stable_z(block, floor))))
    set_default_camera()
    dump_world()
    saved_world = WorldSaver()

    teleport = False
    fixed = [floor] # TODO: get fixed objects
    grasp_gen = get_grasp_gen(robot, 'top')
    ik_fn = get_ik_fn(robot, fixed=fixed, teleport=teleport)
    free_motion_fn = get_free_motion_gen(robot, fixed=fixed, teleport=teleport)
    holding_motion_fn = get_holding_motion_gen(robot, fixed=fixed, teleport=teleport)

    pose0 = BodyPose(block)
    conf0 = BodyConf(robot)
    for grasp, in grasp_gen(block):
        saved_world.restore()
        result1 = ik_fn(block, pose0, grasp)
        if result1 is None:
            continue
        conf1, path2 = result1
        result2 = free_motion_fn(conf0, conf1)
        if result2 is None:
            continue
        path1, = result2
        result3 = holding_motion_fn(conf1, conf0, block, grasp)
        if result3 is None:
            continue
        path3, = result3
        command = Command(path1.body_paths +
                          path2.body_paths +
                          path3.body_paths)
        break
    else:
        print('Unable to find a plan!')
        return

    saved_world.restore()
    update_state()
    # TODO: maybe some OpenRAVE links are disabled
    # http://openrave.org/docs/0.8.2/collada_robot_extensions/

    #< extra
    #type = "collision" >
    #< technique
    #profile = "OpenRAVE"
    # ignore_link_pair

    display = 'control' # control | execute | step
    input('{}?'.format(display))
    if display == 'control':
        enable_gravity()
        command.control(real_time=False, dt=0)
    elif display == 'execute':
        command.refine(num_steps=10).execute(time_step=0.005)
    elif display == 'step':
        command.step()
    else:
        raise ValueError(display)

    print('Quit?')
    wait_for_interrupt()
    disconnect()

if __name__ == '__main__':
    main()