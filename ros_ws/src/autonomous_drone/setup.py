from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'autonomous_drone'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['tests']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('../../launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('autonomous_drone/perception/*.npz')),
        (os.path.join('share', package_name, 'config'), glob('autonomous_drone/bridges/*.npz')),
        # Install executable scripts to lib/<package_name>/ for ROS2 discovery
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Autonomous drone with object detection, avoidance, and following',
    license='MIT',
    tests_require=['pytest'],
)
