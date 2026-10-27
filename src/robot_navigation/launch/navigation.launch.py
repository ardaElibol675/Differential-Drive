import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    robot_navigation_share = get_package_share_directory('robot_navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(robot_navigation_share, 'maps', 'road.yaml'),
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(robot_navigation_share, 'config', 'nav2_params.yaml'),
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    # AMCL initial pose follows the spawn pose
    params_file = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={
            'use_sim_time': use_sim_time,
            'amcl.ros__parameters.initial_pose.x': LaunchConfiguration('x_pose'),
            'amcl.ros__parameters.initial_pose.y': LaunchConfiguration('y_pose'),
            'amcl.ros__parameters.initial_pose.yaw': LaunchConfiguration('yaw'),
        },
        convert_types=True,
    )
    configured_params = ParameterFile(params_file, allow_substs=True)

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Same node set as Humble's navigation_launch.py, but every Nav2 velocity output
    # goes through /cmd_vel_raw -> velocity_smoother -> /cmd_vel_nav -> mode_manager.
    tf_remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    def nav2_node(package, executable, extra_remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output='screen',
            parameters=[configured_params],
            remappings=tf_remappings + list(extra_remappings),
        )

    nav_nodes = [
        nav2_node('nav2_controller', 'controller_server', [('cmd_vel', 'cmd_vel_raw')]),
        nav2_node('nav2_smoother', 'smoother_server'),
        nav2_node('nav2_planner', 'planner_server'),
        nav2_node('nav2_behaviors', 'behavior_server', [('cmd_vel', 'cmd_vel_raw')]),
        nav2_node('nav2_bt_navigator', 'bt_navigator'),
        nav2_node('nav2_waypoint_follower', 'waypoint_follower'),
        nav2_node(
            'nav2_velocity_smoother', 'velocity_smoother',
            [('cmd_vel', 'cmd_vel_raw'), ('cmd_vel_smoothed', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        map_arg,
        params_file_arg,
        use_sim_time_arg,
        x_pose_arg,
        y_pose_arg,
        yaw_arg,
        localization,
        *nav_nodes,
    ])
