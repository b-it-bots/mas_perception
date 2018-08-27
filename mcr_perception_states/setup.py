# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_perception_states'],
    package_dir={'mcr_perception_states': 'ros/src/mcr_perception_states'}
)

setup(**d)
