import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_table_spacemouse_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml') + glob('config/*.sudoers')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dattelpalmara',
    maintainer_email='tamara.bergerhoff@mainfranken-racing.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'spacemouse_teleop = robot_table_spacemouse_teleop.spacemouse_teleop:main'
        ],
    },
)
