from setuptools import find_packages, setup

package_name = 'ican_see'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ultralytics', 'opencv-python', 'cv-bridge'],
    zip_safe=True,
    maintainer='fire',
    maintainer_email='lord.daniel.w@gmail.com',
    description='Vision processing for I_CAN Robot - YOLO object detection',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_server_node = ican_see.yolo_server_node:main',
            'cam_node_opencv = ican_see.cam_node_opencv:main',
            'cam_node_ffmpeg = ican_see.cam_node_ffmpeg:main',
            'cam_node_robust = ican_see.cam_node_robust:main',
            'cam_node_gstreamer = ican_see.cam_node_gstreamer:main',
        ],
    },
)
