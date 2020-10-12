from setuptools import setup
from glob import glob
import os

package_name = 'march_example_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=['src'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
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
            'example_node = src.march_example_node:main'
        ],
    },
)
