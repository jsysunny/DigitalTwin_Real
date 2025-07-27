from setuptools import setup
import os
from glob import glob

package_name = 'lane_follower_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='HSV 기반의 차선 추종 노드',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lane_follower = lane_follower_pkg.lane_follower_hsv:main'
        ],
    },
)

