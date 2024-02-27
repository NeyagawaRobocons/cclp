from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cclp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('./yaml/*.yaml')),
        (os.path.join('share', package_name), glob('./rviz/*.rviz')),
        (os.path.join('share', package_name), glob('./csv/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='paripal',
    maintainer_email='paripal5z34@gmail.com',
    description='coordinate correction',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_map_server_node = cclp.line_map_server_node:main',
        ],
    },
)
