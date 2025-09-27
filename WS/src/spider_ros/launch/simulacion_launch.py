import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_spider_ros = get_package_share_directory('spider_ros')
    robot_description_path = os.path.join(pkg_spider_ros, 'urdf', 'spider_model.urdf.xacro')
    entity_name = 'spider_model_entity'
    
    # 1. Procesar el archivo Xacro a URDF
    robot_description_content = Command(['xacro ', robot_description_path])
    
    # Parámetros del robot
    robot_params = {'robot_description': robot_description_content}
    
    # 2. Iniciar Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), 
            '/gazebo.launch.py'
        ]),
        launch_arguments={'world': 'worlds/empty.world', 
                          'use_sim_time': 'true'}.items(),
    )

    # 3. Publicador de Estados del Robot (TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[robot_params],
    )

    # 4. Inyectar el Modelo en Gazebo (spawn_entity)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', entity_name,
                   '-x', '0', 
                   '-y', '0', 
                   '-z', '2'], 
        output='screen'
    )
    
    # 5. Cargar y Activar el Controlador de Posición
    load_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'spider_position_controller', '--controller-manager', 'controller_manager'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        load_controller
    ])
