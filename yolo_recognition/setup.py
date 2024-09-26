from setuptools import find_packages, setup

package_name = 'yolo_recognition'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(package_name, "yolo_recognition/include"),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sadeep',
    maintainer_email='sadeepm20@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inference_node = yolo_recognition.inference_node:main',
            'lidar_subscriber = yolo_recognition.lidar_subscriber:main',
            'yolo_node = yolo_recognition.yolo_node:main',
        ],
    },
)
