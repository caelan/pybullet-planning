#!/usr/bin/env python

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
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--arms', nargs='+', type=str,
                        default=ARMS, choices=ARMS, #required=True,
                        help='Which arms to compile')
    args = parser.parse_args()
    sys.argv[:] = sys.argv[:1] + ['build'] # Must come after argparse

    for arm in args.arms:
        compile_ikfast(module_name='ik{}'.format(arm.capitalize()), # TODO: {}_arm_ik
                       cpp_filename='{}_arm_ik.cpp'.format(arm)) # TODO: {}_arm.cpp

if __name__ == '__main__':
    main()
