import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_gazebo = get_package_share_directory('navigrid_gazebo')
    pkg_description = get_package_share_directory('navigrid_description')

    world_path = os.path.join(pkg_gazebo, 'worlds', 'navigrid_arena.sdf')
    bridge_config_path = os.path.join(pkg_gazebo, 'config', 'ros_gz_bridge.yaml')
    xacro_path = os.path.join(pkg_description, 'urdf', 'amr.urdf.xacro')
    rviz_config_path = os.path.join(pkg_description, 'rviz', 'navigrid.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('rviz', default='true')
    headless = LaunchConfiguration('headless', default='false')

    # Robot State Publisher
    robot_description_cmd = Command(['xacro ', xacro_path])
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_cmd,
            'use_sim_time': use_sim_time
        }]
    )

    # Launch Gazebo Sim
    gz_args = ['-r ', world_path]
    gazebo_process = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # Spawn AMR in Gazebo at Start Zone A (-12, -12, 0.2)
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'navigrid_amr',
            '-topic', 'robot_description',
            '-x', '-12.0',
            '-y', '-12.0',
            '-z', '0.2',
            '-Y', '0.785'  # facing diagonally towards arena
        ]
    )

    # ROS-GZ Bridge
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={bridge_config_path}'
        ]
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz visualization'),
        DeclareLaunchArgument('headless', default_value='false', description='Run headless simulation'),
        robot_state_publisher_node,
        gazebo_process,
        spawn_robot_node,
        bridge_node,
        rviz_node
    ])
