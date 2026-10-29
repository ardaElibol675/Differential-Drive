import logging
import os
import time

import launch.logging
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

STARTUP_TIMEOUT = 60.0
LIFECYCLE_FAILED = b'Failed to bring up all requested nodes'


def is_true(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def startup_steps(context):
    """(label, process name prefix, output markers or None for process start)"""
    steps = [
        ('Gazebo', 'spawn_entity.py', (b'Calling service /spawn_entity',)),
        ('Robot', 'spawn_entity.py', (b'Successfully spawned entity',)),
        ('Mode manager', 'mode_manager', (b'mode_manager started',)),
    ]
    if is_true(context, 'use_teleop'):
        steps.append(('Control panel', 'control_panel', None))
    if is_true(context, 'use_rviz'):
        steps.append(('RViz', 'rviz2', None))
    if is_true(context, 'slam'):
        steps.append(('SLAM', 'async_slam_toolbox_node', (b'Registering sensor',)))
    else:
        # Separate markers: lifecycle_manager wraps the message in ANSI color codes
        steps += [
            ('Localization', 'lifecycle_manager',
             (b'[lifecycle_manager_localization]', b'Managed nodes are active')),
            ('Navigation', 'lifecycle_manager',
             (b'[lifecycle_manager_navigation]', b'Managed nodes are active')),
        ]
    return steps


class StartupMonitor:
    """Prints one line per finished startup step, then a single ready/not-ready line."""

    def __init__(self, steps):
        self.pending = list(steps)
        self.total = len(steps)
        self.start = time.monotonic()
        self.shutting_down = False
        self.logger = launch.logging.get_logger('bringup')

    def _done(self, step):
        self.pending.remove(step)
        self.logger.info(f'[{self.total - len(self.pending)}/{self.total}] {step[0]} ready')
        if not self.pending:
            self.logger.info(f'SYSTEM READY ({time.monotonic() - self.start:.1f} s)')

    def on_start(self, event, context):
        for step in list(self.pending):
            if step[2] is None and event.process_name.startswith(step[1]):
                self._done(step)

    def on_output(self, event):
        for step in list(self.pending):
            if (step[2] is not None and event.process_name.startswith(step[1])
                    and all(marker in event.text for marker in step[2])):
                self._done(step)
        if LIFECYCLE_FAILED in event.text:
            self.logger.error(
                f'Nav2 bringup failed, logs: {launch.logging.launch_config.log_dir}')

    def on_exit(self, event, context):
        if not self.shutting_down and event.process_name.startswith('spawn_entity.py') \
                and event.returncode != 0:
            self.logger.error('Robot spawn failed (a leftover gzserver may be running)')

    def on_shutdown(self, event, context):
        self.shutting_down = True

    def on_timeout(self, context):
        if self.pending and not self.shutting_down:
            missing = ', '.join(step[0] for step in self.pending)
            self.logger.warning(
                f'SYSTEM NOT READY: not up within {STARTUP_TIMEOUT:.0f} s: {missing}. '
                f'Logs: {launch.logging.launch_config.log_dir}')


def hide_process_lifecycle(record):
    # Drops launch's per-process "started"/"finished cleanly" lines; errors still pass
    return record.levelno >= logging.WARNING or not record.getMessage().startswith(
        ('process started', 'process has finished cleanly', 'sending signal'))


def setup_startup_output(context):
    if not is_true(context, 'show_logs'):
        # Read by every ExecuteProcess created after this point, i.e. all included launch files
        context.environment['OVERRIDE_LAUNCH_PROCESS_OUTPUT'] = 'own_log'
        launch.logging.launch_config.get_screen_handler().addFilter(hide_process_lifecycle)

    monitor = StartupMonitor(startup_steps(context))
    return [
        RegisterEventHandler(OnProcessStart(on_start=monitor.on_start)),
        RegisterEventHandler(OnProcessIO(
            on_stdout=monitor.on_output, on_stderr=monitor.on_output)),
        RegisterEventHandler(OnProcessExit(on_exit=monitor.on_exit)),
        RegisterEventHandler(OnShutdown(on_shutdown=monitor.on_shutdown)),
        TimerAction(period=STARTUP_TIMEOUT, actions=[OpaqueFunction(function=monitor.on_timeout)]),
    ]


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
    show_logs_arg = DeclareLaunchArgument(
        'show_logs', default_value='false',
        description='true: print all node output to the terminal (default: only ~/.ros/log)',
    )
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    # Nodes declared here are created before setup_startup_output runs, so they need it explicitly
    node_output = PythonExpression(
        ["'screen' if '", LaunchConfiguration('show_logs'), "'.lower() == 'true' else 'own_log'"])

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
        output=node_output,
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    mode_manager_node = Node(
        package='robot_tools',
        executable='mode_manager',
        name='mode_manager',
        parameters=[os.path.join(robot_bringup_share, 'config', 'mode_manager.yaml')],
        output=node_output,
    )

    # Always on screen: the [MODE] and z (pose) prints go to this terminal
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
        show_logs_arg,
        x_pose_arg,
        y_pose_arg,
        yaw_arg,
        OpaqueFunction(function=setup_startup_output),
        sim,
        slam,
        navigation,
        rviz_node,
        mode_manager_node,
        control_panel_node,
    ])
