#!/usr/bin/env python

import os, shutil
import fnmatch

from distutils.dir_util import copy_tree
from distutils.core import setup, Extension

# Build C++ extension by running: 'python setup.py build'
# see: https://docs.python.org/3/extending/building.html

# lib name template: 'ikfast_<robot name>'
IKFAST = 'ikfast_abb_irb6600_track'

ikfast_module = Extension(IKFAST,
                          sources=['ikfast0x1000004a.Transform6D.1_2_3_4_5_6_f0.cpp'])

setup(name=IKFAST,
      version='1.0',
      description="ikfast module for abb irb6600 on a linear track.",
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
    import ikfast_abb_irb6600_track
    print('\nikfast module {} imported successful'.format(IKFAST))
except ImportError as e:
    print('\nikfast module {} imported failed'.format(IKFAST))
    raise e
