from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'data_collection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    # find_packages() automatically finds your 'data_collection_pkg' python module
    packages=find_packages(), 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # THIS TELLS ROS2 WHERE YOUR LAUNCH FILES ARE
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Camera data collection and calibration package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    
    # THIS TELLS ROS2 HOW TO CREATE EXECUTABLES FROM YOUR NODES
    entry_points={
        'console_scripts': [
            # format: 'executable_name = package_name.folder.file_name:main_function'
            'usb_camera_node = data_collection_pkg.nodes.usb_camera_node:main',
            'calibration_node = data_collection_pkg.nodes.calibration_node:main',
            'writer_node = data_collection_pkg.nodes.writer_node:main',
            'session_node = data_collection_pkg.nodes.session_node:main',
        ],
    },
)