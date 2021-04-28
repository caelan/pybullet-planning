#!/usr/bin/env python

from __future__ import print_function

import sys
import os
sys.path.append(os.path.join(os.pardir, os.pardir, os.pardir))

from pybullet_tools.ikfast.compile import compile_ikfast

# Build C++ extension by running: 'python setup.py'
# see: https://docs.python.org/3/extending/building.html

def main():
    # lib name template: 'ikfast_<robot name>'
    sys.argv[:] = sys.argv[:1] + ['build']
    robot_name = 'panda_arm'
    compile_ikfast(module_name='ikfast_{}'.format(robot_name),
                   cpp_filename='ikfast_{}.cpp'.format(robot_name))

if __name__ == '__main__':
    main()
