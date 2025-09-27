import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Obtener la ruta del paquete
    pkg_spider_ros = get_package_share_directory('spider_ros')
    
    # Path al archivo URDF/Xacro
    robot_description_path = os.path.join(pkg_spider_ros, 'urdf', 'spider_model.urdf.xacro')
    
    # Path al archivo de configuración de control
    controller_config_path = os.path.join(pkg_spider_ros, 'config', 'spider_control.yaml')
    
    # Cargar el contenido del URDF
    robot_description_content = Command(['xacro ', robot_description_path])
    
    # 1. Nodo del Servidor Gazebo (Fortress)
    # Incluimos el launch de Gazebo para iniciar la simulación.
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), 
            '/gazebo.launch.py'
        ]),
        # Definimos el mundo vacío
        launch_arguments={'world': 'worlds/empty.world'}.items(),
    )

    # 2. Nodo de Publicación del Modelo (spawn_entity)
    # Agrega el modelo al mundo de Gazebo.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'spider_model_entity'],
        output='screen'
    )
    
    # 3. Nodo de Publicación del Estado del Robot (robot_state_publisher)
    # Publica las transformaciones (TF) de los enlaces del robot.
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_content}],
    )

    # 4. Nodo de Control (controller_manager)
    # Carga y configura los controladores de ros2_control.
    load_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 
             'spider_position_controller', '--controller-manager', 'controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        # Declaración de argumentos (opcional, pero buena práctica)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
            
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        load_controller
    ])