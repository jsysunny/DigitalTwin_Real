from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'aruco_yolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/aruco_yolo']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # ✅ 여기 포함되어야 함
        (os.path.join('share', package_name), ['package.xml']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_yolo.aruco_detector:main',
            'aruco_move = aruco_yolo.aruco_move:main',
            'compressed_image_pub = aruco_yolo.compressed_image_pub:main',
            'yolo_detector = aruco_yolo.yolo_detector:main',
            'camera_pub = aruco_yolo.camera_pub:main',
            'pick_and_place = aruco_yolo.pick_and_place:main',
            'moveit_client = aruco_yolo.moveit_client:main',
        ],
    },
    package_data={
        package_name: [
            'config/calibration_params.yaml', 
            'models/yolov8s_trained.pt', 
        ],
    },
  
)
