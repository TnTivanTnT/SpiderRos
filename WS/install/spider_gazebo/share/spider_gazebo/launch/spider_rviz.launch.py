from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('spider_gazebo')
    urdf_file = os.path.join(pkg_share, 'urdf', 'spider_model.urdf.xacro')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        # Tu nodo que publica los estados de las articulaciones
        Node(
            package='spider_gazebo',
            executable='joint_publisher',
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'spider'],
            output='screen'
        ),
        # Nodo RViz (opcional, para lanzar RViz desde el launch)
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
