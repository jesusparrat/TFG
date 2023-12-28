from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='msijesus',
    maintainer_email='jesuspr98@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'talker_node = robot.talker:main',
            # 'subscriber_node = robot.listener:main',
            'driving_robot = robot.driving_robot:main',
            'driving_node = robot.driving_node:main',
            'go_to_goal_node = robot.go_to_goal:main',
            'video_save_node = robot.video_saver:main',
            'maze_solver_node = robot.maze_solver:main',

        ],
    },
)
