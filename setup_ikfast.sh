#!/usr/bin/env bash

# TODO: specify the python version
(cd pybullet_tools/ikfast/franka_panda && python setup.py)
(cd pybullet_tools/ikfast/movo && python setup.py)
(cd pybullet_tools/ikfast/pr2 && python setup.py)
