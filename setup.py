#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
   version='0.0.1',
   scripts=['src/visualstates/main.py'],
   packages=['visualstates'],
   package_dir={'': 'src'}
)

setup(**d)