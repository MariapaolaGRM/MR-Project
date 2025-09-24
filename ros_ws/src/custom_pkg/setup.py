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
        # Aggiunta riga per lettura file launch
        (os.path.join('share/', package_name, 'launch'), glob('launch/*')), 
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
            'odom_to_base_broadcaster = custom_pkg.odom_to_base_broadcaster:main',
        ],
    },
)
