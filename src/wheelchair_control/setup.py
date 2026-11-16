import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'wheelchair_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mariana',
    maintainer_email='mariana@example.com',
    description='Nos de controle da cadeira de rodas robotizada.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_keyboard = wheelchair_control.teleop_keyboard:main',
            'joystick_teleop = wheelchair_control.joystick_teleop:main',
            'safety_arbiter  = wheelchair_control.safety_arbiter:main',
            'scan_filter     = wheelchair_control.scan_filter:main',
            'head_teleop     = wheelchair_control.head_teleop:main',
            'head_control    = wheelchair_control.head_control:main',
            'obstacle_mover  = wheelchair_control.obstacle_mover:main',
            'metrics_logger  = wheelchair_control.metrics_logger:main',
            'analyze_runs    = wheelchair_control.analyze_runs:main',
        ],
    },
)
