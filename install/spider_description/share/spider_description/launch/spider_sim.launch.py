"""
spider_sim.launch.py
--------------------
Launch file for Spider Quadruped Robot - Hito 1 (Structural)

Starts:
  1. Ignition Gazebo 6 with empty world (running, no pause)
  2. robot_state_publisher with URDF from xacro
  3. Spawn robot via ros_gz_sim (TimerAction +3s)
  4. Activate joint_state_broadcaster (TimerAction +5s)

Arguments:
  use_sim_time  (bool, default: true)  - Use simulation clock
  gz_headless   (bool, default: false) - Launch Gazebo without GUI
  spawn_z       (float, default: 0.25) - Spawn height above ground
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = get_package_share_directory('spider_description')

    # ---------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    gz_headless_arg = DeclareLaunchArgument(
        'gz_headless',
        default_value='false',
        description='Launch Ignition Gazebo without GUI (server mode)'
    )
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.25',
        description='Z position for robot spawn (meters above ground)'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    gz_headless   = LaunchConfiguration('gz_headless')
    spawn_z       = LaunchConfiguration('spawn_z')

    # ---------------------------------------------------------------
    # Paths
    # ---------------------------------------------------------------
    urdf_path   = os.path.join(pkg_share, 'urdf', 'spider.urdf.xacro')
    world_path  = os.path.join(pkg_share, 'worlds', 'empty.sdf')

    # ---------------------------------------------------------------
    # 1. Ignition Gazebo — with GUI
    # ---------------------------------------------------------------
    gazebo_gui = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path],
        output='screen',
        condition=UnlessCondition(gz_headless),
    )

    # 1b. Ignition Gazebo — headless (server only)
    gazebo_headless = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', '-s', world_path],
        output='screen',
        condition=IfCondition(gz_headless),
    )

    # ---------------------------------------------------------------
    # 2. robot_state_publisher
    # ---------------------------------------------------------------
    robot_description = Command([
        FindExecutable(name='xacro'), ' ', urdf_path
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # ---------------------------------------------------------------
    # 3. Spawn robot — delayed 3 s to let Gazebo finish loading
    # ---------------------------------------------------------------
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_spider',
                output='screen',
                arguments=[
                    '-name', 'spider',
                    '-topic', 'robot_description',
                    '-x', '0',
                    '-y', '0',
                    '-z', spawn_z,
                    '-R', '0',
                    '-P', '0',
                    '-Y', '0',
                ],
            )
        ]
    )

    # ---------------------------------------------------------------
    # 4. Activate joint_state_broadcaster — delayed 5 s after launch
    #    (after spawn + controller_manager is up)
    # ---------------------------------------------------------------
    activate_jsb = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'control', 'load_controller',
                    '--set-state', 'active',
                    'joint_state_broadcaster',
                ],
                output='screen',
            )
        ]
    )

    # ---------------------------------------------------------------
    # ROS <-> Ignition bridge for /clock (sim time)
    # ---------------------------------------------------------------
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_ros_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        gz_headless_arg,
        spawn_z_arg,
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        gz_ros_bridge,
        spawn_robot,
        activate_jsb,
    ])
