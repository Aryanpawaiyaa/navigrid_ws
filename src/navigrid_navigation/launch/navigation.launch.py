import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = get_package_share_directory('navigrid_navigation')
    planner_cfg = os.path.join(pkg_nav, 'config', 'planner_params.yaml')
    safety_cfg = os.path.join(pkg_nav, 'config', 'safety_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    costmap_node = Node(
        package='navigrid_navigation',
        executable='costmap_generator',
        name='costmap_generator',
        output='screen',
        parameters=[planner_cfg, {'use_sim_time': use_sim_time}]
    )

    planner_node = Node(
        package='navigrid_navigation',
        executable='adaptive_global_planner',
        name='adaptive_global_planner',
        output='screen',
        parameters=[planner_cfg, {'use_sim_time': use_sim_time}]
    )

    controller_node = Node(
        package='navigrid_navigation',
        executable='local_reactive_controller',
        name='local_reactive_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    safety_node = Node(
        package='navigrid_navigation',
        executable='safety_override_node',
        name='safety_override_node',
        output='screen',
        parameters=[safety_cfg, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        costmap_node,
        planner_node,
        controller_node,
        safety_node
    ])
