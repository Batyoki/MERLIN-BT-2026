import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'controller'

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
        # dynamics.resolve_urdf() looks here; replaces the /home/sp/... paths
        (os.path.join('share', package_name, 'urdf'),
            ['controller/rx200.urdf', 'controller/vx300s.urdf']),
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='sp',
    maintainer_email='cs23bt072@iitdh.ac.in',
    description='MERLIN bilateral teleoperation controller (MISA + MPC + energy tank)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller = controller.controller:main',
        ],
    },
)
