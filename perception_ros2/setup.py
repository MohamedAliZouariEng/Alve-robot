from setuptools import setup
import os
from glob import glob

package_name = 'perception_ros2'

def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join(path, filename))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Add this line to include .py launch files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),  # Add this line to include .rviz files
        (os.path.join('share', package_name, 'models/demo_cube'), package_files('models/demo_cube')),
        (os.path.join('share', package_name, 'models/apple'), package_files('models/apple')),
        (os.path.join('share', package_name, 'models/banana'), package_files('models/banana')),
        (os.path.join('share', package_name, 'models/tennis_ball'), package_files('models/tennis_ball')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'range_detector = perception_ros2.range_detector:main',
        ],
    },
)
