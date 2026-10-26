import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger


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

        self.create_timer(1.0 / 20.0, self.timer_cb)

        self.publish_mode()
        self.get_logger().info(f'mode_manager started, initial mode = {self.mode}')

    def teleop_cb(self, msg):
        self.last_teleop_twist = msg
        self.last_teleop_time = self.get_clock().now()

    def nav_cb(self, msg):
        self.last_nav_twist = msg
        self.last_nav_time = self.get_clock().now()

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
            self.cancel_active_nav_goal()

        response.success = True
        response.message = self.mode
        return response

    def cancel_active_nav_goal(self):
        if not self.cancel_client.service_is_ready():
            self.get_logger().warn('cancel_goal servisi yok, Nav2 hedefi iptal edilmedi')
            return
        self.cancel_client.call_async(CancelGoal.Request())


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
