import pybullet as p
import os
import argparse

from pybullet_utils import connect, add_data_path, disconnect, get_pose, get_body_names, body_from_name
from problems import holding_problem

def main():
    parser = argparse.ArgumentParser()  # Automatically includes help
    parser.add_argument('-viewer', action='store_true', help='enable viewer.')
    args = parser.parse_args()

    connect(use_gui=args.viewer)
    add_data_path()

    problem = holding_problem()
    print get_body_names()
    pr2 = body_from_name('pr2')
    print get_pose(pr2)

    #path = os.path.join('worlds', 'test_ss')
    #p.saveWorld(path)
    #state_id = p.saveState()
    #p.saveBullet(path)

    p.stepSimulation()
    raw_input('Finish?')
    disconnect()


if __name__ == '__main__':
    main()