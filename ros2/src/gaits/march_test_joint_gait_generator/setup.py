import os
from glob import glob
from setuptools import setup

package_name = 'march_test_joint_gait_generator'


def data_files():
    data = [
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
         (os.path.join('share', package_name, 'launch'),
          glob('launch/*.launch.py'))
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
    description='Gait generator for the test_joint',
    license='TODO: License declaration',
    tests_require=['pytest', 'urfdom_py', 'unittest', 'parameterized'],
    entry_points={
        'console_scripts': [
            f'generate_test_joint_gait = {package_name}.test_joint_gait_generator:main'
        ],
    },
)
