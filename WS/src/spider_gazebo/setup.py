from setuptools import setup
import os
from glob import glob

package_name = 'spider_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Añade esta línea para instalar la carpeta launch
        ('share/spider_gazebo/launch', glob('launch/*.launch.py')),
        ('share/spider_gazebo/urdf', glob('urdf/*.urdf.xacro')),  
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='ivanmorladag@gmail.com',
    description='Spider gazebo package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_publisher = spider_gazebo.joint_publisher:main',
        ],
    },
)
