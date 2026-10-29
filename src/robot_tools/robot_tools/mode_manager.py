import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
                       qos_profile_action_status_default)

from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_srvs.srv import Trigger

ACTIVE_STATUSES = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                   GoalStatus.STATUS_CANCELING)


class ModeManager(Node):

    def __init__(self):
        super().__init__('mode_manager')

        self.declare_parameter('initial_mode', 'MANUAL')
        self.declare_parameter('cmd_timeout', 0.5)
        self.declare_parameter('cancel_nav_on_manual', True)

        initial_mode = self.get_parameter('initial_mode').get_parameter_value().string_value.upper()
        self.mode = initial_mode if initial_mode in ('MANUAL', 'AUTO') else 'MANUAL'
        self.cmd_timeout = self.get_parameter('cmd_timeout').get_parameter_value().double_value
        self.cancel_nav_on_manual = self.get_parameter(
            'cancel_nav_on_manual').get_parameter_value().bool_value

        self.last_teleop_twist = Twist()
        self.last_teleop_time = None
        self.last_nav_twist = Twist()
        self.last_nav_time = None
        # Nav2 goal tracking, keyed by goal UUID bytes
        self.goal_status = {}
        self.goal_infos = {}
        self.current_goal = None
        self.current_goal_planned = False
        self.mode_canceled_goals = set()
        # Goal planned in MANUAL: canceled in Nav2 and resent on the switch to AUTO
        self.held_goal = None
        self.held_goal_ids = set()
        self.resume_pending = False

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        mode_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.mode_pub = self.create_publisher(String, 'robot_mode', mode_qos)

        self.create_subscription(Twist, 'cmd_vel_teleop', self.teleop_cb, 10)
        self.create_subscription(Twist, 'cmd_vel_nav', self.nav_cb, 10)

        self.create_service(Trigger, '~/toggle_mode', self.toggle_mode_cb)
        self.cancel_client = self.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self.nav_status_cb, qos_profile_action_status_default)
        self.create_subscription(Path, '/plan', self.plan_cb, 1)
        # Nav2 leaves the last plan in RViz when a goal ends; overwrite it with an empty path
        self.plan_pub = self.create_publisher(Path, '/plan', 1)
        # bt_navigator turns poses on /goal_pose into navigate_to_pose goals
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 1)
        self.nav_status_pub = self.create_publisher(String, 'nav_status', 10)

        self.create_timer(1.0 / 20.0, self.timer_cb)

        self.publish_mode()
        self.get_logger().info(f'mode_manager started, initial mode = {self.mode}')

    def teleop_cb(self, msg):
        self.last_teleop_twist = msg
        self.last_teleop_time = self.get_clock().now()

    def nav_cb(self, msg):
        self.last_nav_twist = msg
        self.last_nav_time = self.get_clock().now()

    def nav_status_cb(self, msg):
        statuses = {}
        for status in msg.status_list:
            goal_id = bytes(status.goal_info.goal_id.uuid)
            statuses[goal_id] = status.status
            self.goal_infos[goal_id] = status.goal_info
        # Starts before ends: Nav2 aborts a preempted goal once the new one is accepted
        for goal_id, status in statuses.items():
            if goal_id not in self.goal_status and status in ACTIVE_STATUSES:
                self.on_goal_started(goal_id)
        for goal_id, status in statuses.items():
            old = self.goal_status.get(goal_id)
            if old in ACTIVE_STATUSES and status not in ACTIVE_STATUSES:
                self.on_goal_ended(goal_id, status)
        self.goal_status = statuses
        self.goal_infos = {k: v for k, v in self.goal_infos.items() if k in statuses}

    def on_goal_started(self, goal_id):
        self.current_goal = goal_id
        if self.resume_pending:
            self.resume_pending = False
            self.current_goal_planned = True
            self.notify('Driving started')
        else:
            self.current_goal_planned = False
            self.notify('Goal received')

    def on_goal_ended(self, goal_id, status):
        if goal_id in self.held_goal_ids:
            self.held_goal_ids.discard(goal_id)
            return
        canceled_by_mode = goal_id in self.mode_canceled_goals
        self.mode_canceled_goals.discard(goal_id)
        if goal_id != self.current_goal:
            # Replaced by a newer goal
            return
        self.current_goal = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.notify('Goal reached')
        elif status == GoalStatus.STATUS_CANCELED:
            self.notify('Route canceled (MANUAL)' if canceled_by_mode
                        else 'Route canceled')
        elif self.current_goal_planned:
            self.notify('Navigation failed')
        else:
            self.notify('No route found')
        self.clear_plan()

    def plan_cb(self, msg):
        goal_id = self.current_goal
        if not msg.poses or goal_id is None or self.current_goal_planned \
                or goal_id in self.held_goal_ids:
            return
        self.current_goal_planned = True
        if self.mode == 'AUTO':
            self.notify('Route created, driving')
            return
        # MANUAL: keep the route on screen, but stop Nav2 before its progress checker fails it
        self.held_goal = PoseStamped(header=msg.header, pose=msg.poses[-1].pose)
        self.held_goal_ids.add(goal_id)
        self.cancel_nav_goals(self.goal_infos[goal_id])
        self.notify('Route created, waiting for AUTO')

    def notify(self, text):
        self.get_logger().info(f'navigation: {text}')
        self.nav_status_pub.publish(String(data=text))

    def clear_plan(self):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.plan_pub.publish(msg)

    def timer_cb(self):
        if self.mode == 'MANUAL':
            twist, last_time = self.last_teleop_twist, self.last_teleop_time
        else:
            twist, last_time = self.last_nav_twist, self.last_nav_time

        if last_time is not None:
            age = (self.get_clock().now() - last_time).nanoseconds / 1e9
        else:
            age = float('inf')

        self.cmd_vel_pub.publish(twist if age <= self.cmd_timeout else Twist())

    def publish_mode(self):
        msg = String()
        msg.data = self.mode
        self.mode_pub.publish(msg)

    def toggle_mode_cb(self, request, response):
        old_mode = self.mode
        self.mode = 'AUTO' if self.mode == 'MANUAL' else 'MANUAL'

        self.cmd_vel_pub.publish(Twist())
        self.publish_mode()
        self.get_logger().info(f'mode changed: {old_mode} -> {self.mode}')

        if old_mode == 'AUTO' and self.mode == 'MANUAL' and self.cancel_nav_on_manual:
            self.mode_canceled_goals = {
                goal_id for goal_id, status in self.goal_status.items()
                if status in ACTIVE_STATUSES}
            self.cancel_nav_goals()
        elif self.mode == 'AUTO' and self.held_goal is not None:
            self.resume_held_goal()

        response.success = True
        response.message = self.mode
        return response

    def cancel_nav_goals(self, goal_info=None):
        """Cancel one goal, or all goals when goal_info is None (empty request)."""
        if not self.cancel_client.service_is_ready():
            self.get_logger().warn('cancel_goal service not available, Nav2 goal not canceled')
            return
        request = CancelGoal.Request()
        if goal_info is not None:
            request.goal_info = GoalInfo(goal_id=goal_info.goal_id)
        self.cancel_client.call_async(request)

    def resume_held_goal(self):
        goal, self.held_goal = self.held_goal, None
        goal.header.stamp = self.get_clock().now().to_msg()
        self.resume_pending = True
        self.goal_pub.publish(goal)


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # The SIGINT handler may have already shut the context down
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
