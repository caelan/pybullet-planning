#!/usr/bin/env python2

from __future__ import print_function

import sys
import os
import argparse
sys.path.append(os.path.join(os.pardir, os.pardir, os.pardir))

from pybullet_tools.ikfast.compile import compile_ikfast

# Build C++ extension by running: 'python setup.py'
# see: https://docs.python.org/3/extending/building.html

ARMS = ['left', 'right']

def main():
    # TODO: doesn't currently work with python3
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arm', choices=ARMS, required=True,
                        help='Which arm to compile')
    args = parser.parse_args()
    sys.argv[:] = sys.argv[:1] + ['build']

    arm = args.arm
    compile_ikfast(module_name='movo_{}_arm_ik'.format(arm),
                   cpp_filename='movo_{}_arm_ik.cpp'.format(arm))

if __name__ == '__main__':
    main()
