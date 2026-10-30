from setuptools import find_packages, setup

package_name = 'teleoperation_hopefully'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sp',
    maintainer_email='sp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rx150_fk_node = teleoperation_hopefully.fk:main',
            'vx300s_ik_node = teleoperation_hopefully.ik:main',
            'diff_ik_vx300s_node = teleoperation_hopefully.diff_ik:main',
            'moveit2_ik_vx300s_node = teleoperation_hopefully.move_it_2_ik:main',
        ],
    },
)
