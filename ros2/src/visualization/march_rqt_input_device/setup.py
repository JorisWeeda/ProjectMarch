from setuptools import setup
from glob import glob
import os

package_name = 'march_rqt_input_device'

ros1_source = os.path.join('..', '..', '..', '..', 'ros1', 'src', 'visualization', package_name)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
            [os.path.join('resource', package_name)]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), [os.path.join(ros1_source, 'plugin.xml')]),
        (os.path.join('share', package_name), [os.path.join(ros1_source, 'resource', 'input_device.ui')]),
        (os.path.join('share', package_name, 'resource', 'img'), glob('resource/img/*.png')),
        (os.path.join('share', package_name, 'config'), glob('config/**'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Project March',
    maintainer_email='software@projectmarch.nl',
    description='Developer input device to send commands to the march exoskeleton',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'input_device = march_rqt_input_device.input_device_plugin:main'
        ],
    },
)
