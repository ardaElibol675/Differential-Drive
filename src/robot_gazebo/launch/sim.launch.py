import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_gazebo_share = get_package_share_directory('robot_gazebo')
    robot_description_share = get_package_share_directory('robot_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(robot_gazebo_share, 'worlds', 'road.world'),
    )
    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    verbose_arg = DeclareLaunchArgument('verbose', default_value='true')
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    set_model_path = AppendEnvironmentVariable('GAZEBO_MODEL_PATH', robot_gazebo_share)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'verbose': LaunchConfiguration('verbose'),
        }.items(),
    )

    xacro_file = os.path.join(robot_description_share, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_robot',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', '0.05',
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        verbose_arg,
        x_pose_arg,
        y_pose_arg,
        yaw_arg,
        set_model_path,
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
    ])
