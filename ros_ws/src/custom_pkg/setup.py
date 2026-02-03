from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'custom_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/', package_name, 'launch'), glob('launch/*')),  # Line for reading launch file
        (os.path.join('share/', package_name, 'config'), glob('config/*')), 
        (os.path.join('share/', package_name, 'rviz'), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_static_broadcaster = custom_pkg.map_static_broadcaster:main',
            'yolo_node = custom_pkg.yolo_node:main',
            'central_node = custom_pkg.central_node:main',
            'goal_node = custom_pkg.goal_node:main',
            'test_node = custom_pkg.test_node:main',
        ],
    },
)
