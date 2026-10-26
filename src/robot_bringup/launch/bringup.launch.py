import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_gazebo_share = get_package_share_directory('robot_gazebo')
    robot_navigation_share = get_package_share_directory('robot_navigation')
    robot_bringup_share = get_package_share_directory('robot_bringup')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(robot_gazebo_share, 'worlds', 'road.world'),
    )
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(robot_navigation_share, 'maps', 'road.yaml'),
    )
    slam_arg = DeclareLaunchArgument('slam', default_value='false')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    use_teleop_arg = DeclareLaunchArgument('use_teleop', default_value='true')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(robot_bringup_share, 'rviz', 'nav.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    mode_manager_node = Node(
        package='robot_tools',
        executable='mode_manager',
        name='mode_manager',
        parameters=[os.path.join(robot_bringup_share, 'config', 'mode_manager.yaml')],
    )

    control_panel_node = Node(
        package='robot_tools',
        executable='control_panel',
        name='control_panel',
        parameters=[os.path.join(robot_bringup_share, 'config', 'control_panel.yaml')],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_teleop')),
    )

    return LaunchDescription([
        world_arg,
        map_arg,
        slam_arg,
        use_rviz_arg,
        gui_arg,
        use_teleop_arg,
        sim,
        rviz_node,
        mode_manager_node,
        control_panel_node,
    ])
