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
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Nguyen Pham, Aaron Mundanilkunathil',
    author_email='nmhieu.pham@gmail.com, aaron.mundanilkunathil@sjsu.edu',
    description='Autonomous drone with object detection, avoidance, and following',
    license='MIT',
    tests_require=['pytest'],
)
