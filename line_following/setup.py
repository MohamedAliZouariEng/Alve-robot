from setuptools import find_packages, setup

package_name = 'line_following'

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
    maintainer='ubuntu24',
    maintainer_email='ubuntu24@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_red_line = line_following.follow_red_line:main',
            'follow_red_line_optimized = line_following.follow_red_line_optimized:main',
            'follow_door = line_following.follow_door:main',
        ],
    },
)
