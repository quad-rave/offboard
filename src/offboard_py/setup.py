#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['offboard_py'],
    package_dir={'': 'src'},
    scripts=['bin/offboard_py_node', 'bin/offboard_state_listener_node']
)

setup(**d)
