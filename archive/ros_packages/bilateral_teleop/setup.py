import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bilateral_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sp',
    maintainer_email='sp@todo.todo',
    description='Bilateral teleoperation between ReactorX-150 and ViperX-300s using MoveIt2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # FK/IK nodes
            'rx150_fk = bilateral_teleop.reactor_fk:main',
            'viper_ik = bilateral_teleop.viper_ik:main',
            # FK Publisher node (old combined node)
            'fk_publisher = bilateral_teleop.fk_publisher:main',
            # Separate FK and IK nodes
            'rx150_fk_publisher = bilateral_teleop.rx150_fk_publisher:main',
            'vx300s_ik_subscriber = bilateral_teleop.vx300s_ik_subscriber:main',
            # MoveIt2-based bilateral teleoperation
            'moveit2_bilateral = bilateral_teleop.moveit2_bilateral:main',
            'moveit2_bilateral_direct = bilateral_teleop.moveit2_bilateral:main_direct',
            # Direct joint mapping bilateral teleoperation
            'direct_bilateral = bilateral_teleop.direct_bilateral:main',
            'direct_bilateral_scaled = bilateral_teleop.direct_bilateral:main_scaled',
        ],
    },
)
