from setuptools import find_packages, setup
import os
package_name = 'ar_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ['launch/display.launch.py', 'launch/steer.launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/aruco.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lightprohvet',
    maintainer_email='lightprovhet@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_steering_node = ar_tracking.aruco_steering_node:main',
        ],
    },
)
