from setuptools import setup
from glob import glob
import os

package_name = 'march_gain_scheduling'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project March',
    maintainer_email='software@projectmarch.nl',
    description='A feature package that implements gain scheduling control',
    license='TODO: License declaration',
    tests_require=['nosetests'],
    entry_points={
        'console_scripts': [
            'gain_scheduling_node = march_gain_scheduling.gain_scheduling_node:main'
        ],
    },
)
