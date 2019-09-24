#!/usr/bin/env python2

from __future__ import print_function

import os
import shutil
import fnmatch

from distutils.dir_util import copy_tree
from distutils.core import setup, Extension

# Build C++ extension by running: 'python setup.py build'
# see: https://docs.python.org/3/extending/building.html

# lib name template: 'ikfast_<robot name>'
from utils import IKFAST, CPP_PATH

ikfast_module = Extension(IKFAST, sources=[CPP_PATH])

setup(name=IKFAST,
      version='1.0',
      description="ikfast module for Franka Panda arm.",
      ext_modules=[ikfast_module])

build_lib_path = None
for root, dirnames, filenames in os.walk(os.getcwd()):
    if fnmatch.fnmatch(root, os.path.join(os.getcwd(), "*build", "lib*")):
        build_lib_path = root
        break
assert build_lib_path

copy_tree(build_lib_path, os.getcwd())
shutil.rmtree(os.path.join(os.getcwd(), 'build'))

try:
    import ikfast_franka_panda
    print('\nikfast module {} imported successful'.format(IKFAST))
except ImportError as e:
    print('\nikfast module {} imported failed'.format(IKFAST))
    raise e