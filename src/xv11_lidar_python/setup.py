from setuptools import setup
from glob import glob
import os

package_name = 'xv11_lidar_python'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.[pxy]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niklas',
    maintainer_email='niklas@pirnay.com',
    description='A python driver for the lidar sensors that use the xv_11 firmware version 2, which are prominent in vacuum cleaning robots.',
    license='MIT License',
    entry_points={
        'console_scripts': [
            'xv11_lidar = xv11_lidar_python.xv11_lidar_publisher:main'
        ],
    },
)
