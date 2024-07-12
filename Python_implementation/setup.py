from setuptools import setup
from glob import glob
import os

package_name = 'copub'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'copub.object_detection_publisher',
        'copub.mask_rcnn',
        'copub.object_detection_listener'
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'dnn'), glob('dnn/*')),
    ],
    install_requires=['setuptools', 'opencv-python', 'numpy', 'tensorflow'],
    zip_safe=True,
    maintainer='mayank',
    maintainer_email='mayanka22@iitk.ac.in',
    description='ROS 2 package to publish object distances using Realsense camera and Mask RCNN',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection_publisher = copub.object_detection_publisher:main',
            'object_detection_listener = copub.object_detection_listener:main',
        ],
    },
)

# from setuptools import find_packages, setup

# package_name = 'copub'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=find_packages(exclude=['test']),
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name + '/dnn', [
#             'copub/dnn/frozen_inference_graph_coco.pb',
#             'copub/dnn/mask_rcnn_inception_v2_coco_2018_01_28.pbtxt'
#         ]),
#     ],
#     install_requires=['setuptools', 'opencv-python', 'numpy','tensorflow'],
#     zip_safe=True,
#     maintainer='mayank',
#     maintainer_email='mayanka22@iitk.ac.in',
#     description='TODO: Package description',
#     license='Apache-2.0',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'object_detection_publisher = copub.object_detection_publisher:main',
#             'object_detection_listener = copub.object_detection_listener:main', 
#         ],
#     },
# )
