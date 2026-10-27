import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_gazebo_share, 'launch', 'sim.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_navigation_share, 'launch', 'slam.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    # Delayed so that the sim clock and odom TF exist before AMCL/Nav2 start
    navigation = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_navigation_share, 'launch', 'navigation.launch.py')
                ),
                # params_file is passed explicitly: gazebo.launch.py leaks params_file:=''
                launch_arguments={
                    'map': LaunchConfiguration('map'),
                    'params_file': os.path.join(
                        robot_navigation_share, 'config', 'nav2_params.yaml'),
                    'x_pose': LaunchConfiguration('x_pose'),
                    'y_pose': LaunchConfiguration('y_pose'),
                    'yaw': LaunchConfiguration('yaw'),
                }.items(),
            ),
        ],
        condition=UnlessCondition(LaunchConfiguration('slam')),
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
        x_pose_arg,
        y_pose_arg,
        yaw_arg,
        sim,
        slam,
        navigation,
        rviz_node,
        mode_manager_node,
        control_panel_node,
    ])
