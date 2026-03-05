"""
spider_sim.launch.py
--------------------
Launch file for Spider Quadruped Robot - Hito 3 (IK & Gait)

Starts:
  1. Ignition Gazebo 6 with empty world (running, no pause)
  2. robot_state_publisher with URDF from xacro
  3. Spawn robot via ros_gz_sim (TimerAction +3s)
  4. Activate joint_state_broadcaster (TimerAction +5.5s)
  5. Activate 4 leg position controllers (TimerAction +6.5-7.1s)
  6. [Optional] spider_ik_node (TimerAction +8.5s, if enable_ik_node:=true)

Arguments:
  use_sim_time       (bool,  default: true)   - Use simulation clock
  gz_headless        (bool,  default: false)  - Launch Gazebo without GUI
  spawn_z            (float, default: 0.18)   - Spawn height above ground
  enable_ik_node     (bool,  default: true)   - Launch spider_ik_node (IK/gait)
  step_height        (float, default: 0.05)   - Swing step height (m)
  gait_cycle_time    (float, default: 2.0)    - Full gait cycle duration (s)
  swing_duty_cycle   (float, default: 0.3)    - Fraction of cycle in swing
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
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
from launch_ros.parameter_descriptions import ParameterValue
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
        default_value='0.18',
        description='Z position for robot spawn (meters above ground)'
    )
    enable_ik_node_arg = DeclareLaunchArgument(
        'enable_ik_node',
        default_value='true',
        description='Launch spider_ik_node (IK/gait locomotion). Set false for manual teleop.'
    )
    step_height_arg = DeclareLaunchArgument(
        'step_height',
        default_value='0.05',
        description='Swing step height in metres'
    )
    gait_cycle_time_arg = DeclareLaunchArgument(
        'gait_cycle_time',
        default_value='2.0',
        description='Full gait cycle duration in seconds'
    )
    swing_duty_cycle_arg = DeclareLaunchArgument(
        'swing_duty_cycle',
        default_value='0.3',
        description='Fraction of gait cycle that each leg spends in swing'
    )

    use_sim_time    = LaunchConfiguration('use_sim_time')
    gz_headless     = LaunchConfiguration('gz_headless')
    spawn_z         = LaunchConfiguration('spawn_z')
    enable_ik_node  = LaunchConfiguration('enable_ik_node')
    step_height     = LaunchConfiguration('step_height')
    gait_cycle_time = LaunchConfiguration('gait_cycle_time')
    swing_duty_cycle = LaunchConfiguration('swing_duty_cycle')

    # ---------------------------------------------------------------
    # Make the ign_ros2_control .so visible to Ignition Gazebo.
    # Ignition searches IGN_GAZEBO_SYSTEM_PLUGIN_PATH for system plugins;
    # the .so lives in the ROS2 lib dir which is not on that path by default.
    # ---------------------------------------------------------------
    ign_plugin_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

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
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_path]),
        value_type=str
    )

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
    # 4. Activate joint_state_broadcaster — delayed 5.5 s after launch
    #    (after spawn + controller_manager is fully ready)
    # ---------------------------------------------------------------
    activate_jsb = TimerAction(
        period=5.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    FindExecutable(name='ros2'),
                    'control',
                    'load_controller',
                    '--set-state',
                    'active',
                    'joint_state_broadcaster',
                ],
                output='screen',
            )
        ]
    )

    # ---------------------------------------------------------------
    # 4b. Activate leg position controllers — delayed 6.5 s + increments
    #     (after joint_state_broadcaster is fully up)
    #     Each controller gets a separate load_controller call
    # ---------------------------------------------------------------
    leg_controllers = [
        'front_right_leg_controller',
        'front_left_leg_controller',
        'back_left_leg_controller',
        'back_right_leg_controller',
    ]
    
    # Create TimerActions for each leg controller (6.5s, 6.7s, 6.9s, 7.1s)
    activate_leg_controller_actions = []
    for idx, controller_name in enumerate(leg_controllers):
        delay = 6.5 + (idx * 0.2)  # 6.5, 6.7, 6.9, 7.1 seconds
        activate_leg_controller_actions.append(
            TimerAction(
                period=delay,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            FindExecutable(name='ros2'),
                            'control',
                            'load_controller',
                            '--set-state',
                            'active',
                            controller_name,
                        ],
                        output='screen',
                    )
                ]
            )
        )

    # ---------------------------------------------------------------
    # 5. spider_ik_node — delayed 8.5 s (after all controllers active)
    #    Only started when enable_ik_node:=true (default)
    # ---------------------------------------------------------------
    spider_ik_node = TimerAction(
        period=8.5,
        actions=[
            Node(
                package='spider_description',
                executable='spider_ik_node.py',
                name='spider_ik_node',
                output='screen',
                parameters=[{
                    'use_sim_time':      use_sim_time,
                    'step_height':       step_height,
                    'gait_cycle_time':   gait_cycle_time,
                    'swing_duty_cycle':  swing_duty_cycle,
                }],
                condition=IfCondition(enable_ik_node),
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
        enable_ik_node_arg,
        step_height_arg,
        gait_cycle_time_arg,
        swing_duty_cycle_arg,
        ign_plugin_path,
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        gz_ros_bridge,
        spawn_robot,
        activate_jsb,
    ] + activate_leg_controller_actions + [spider_ik_node])
