import os
from glob import glob
from setuptools import setup
import xacro

package_name = 'march_simulation'
ros1_source = os.path.join('..', '..', '..', '..', 'ros1', 'src', 'simulation',
                           'march_simulation')


def data_files():
    """ Generates the list of data files necessary for gait selection, the gait and subgait files
    required for testing are taken from the ros1 directory to decrease duplication. """
    data = [
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'effort_control'),
         glob(ros1_source + '/config/effort_control/*.yaml')),
        (os.path.join('share', package_name, 'config', 'inertia_control'),
         glob(ros1_source + '/config/inertia_control/*.yaml')),
        (os.path.join('share', package_name, 'config', 'position_control'),
         glob('config/position_control/*.yaml')),
        (os.path.join('share', package_name, 'obstacles'),
         glob(ros1_source + '/obstacles/*.xacro')),
        (os.path.join('share', package_name, 'worlds'),
         glob(ros1_source + '/worlds/*.world')),
    ]
    return data

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project March',
    maintainer_email='software@projectmarch.nl',
    description='The simulation world in which the exoskeleton can be placed',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'to_world_transform = march_simulation.to_world_transform:main',
            'spawn_obstacle = march_simulation.spawn_obstacle:main',
            'example = march_simulation.example_joint_trajectory:main'
        ],
    },
)
