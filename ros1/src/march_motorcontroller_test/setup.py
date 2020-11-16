# ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup_args = generate_distutils_setup(
    packages=['march_motorcontroller_test'],
    package_dir={'': 'src'},
    scripts=['scripts/generate_subgait'],
)

setup(**setup_args)
