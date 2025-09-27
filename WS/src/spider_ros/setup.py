import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'spider_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # FIX: Asegurarse de que los archivos .py dentro de 'launch' sean copiados
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        
        # Archivos URDF/Xacro
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
        # Archivos de configuración
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Paquete de ROS 2 para el control de un robot araña.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Punto de entrada para el nodo de control Python
            'spider_controller = spider_ros.spider_controller:main'
        ],
    },
)
