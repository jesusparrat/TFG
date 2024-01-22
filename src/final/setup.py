from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'final'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*')),
        (os.path.join('share', package_name,'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name,'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name,'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='msijesus',
    maintainer_email='jesuspt98@gmail.com',
    description='setups for the final project of my TFG',
    license='Universidad Complutense de Madrid',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driving_node = final.driving:main' ,
            'go_to_goal = final.go_to_goal:main',
            'video_saver = final.video_saver:main',
            'maze_solver = final.maze_solver:main',
        ],
    },
)
