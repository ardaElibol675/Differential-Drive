import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    robot_navigation_share = get_package_share_directory('robot_navigation')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(robot_navigation_share, 'config', 'slam.yaml'),
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': LaunchConfiguration('slam_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([
        slam_params_arg,
        use_sim_time_arg,
        slam_toolbox,
    ])
