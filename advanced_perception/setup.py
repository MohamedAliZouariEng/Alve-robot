from setuptools import find_packages, setup

package_name = 'advanced_perception'

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
            'yolo_object_detection = advanced_perception.yolo_object_detection:main',
            'yolo_segmentation = advanced_perception.yolo_segmentation:main',
            'fruit_mask_saver = advanced_perception.fruit_mask_saver:main',
        ],
    },
)
