import math
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from PyQt5.QtCore import QEvent, Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QGridLayout, QGroupBox, QHBoxLayout, QLabel, QSlider, QSpinBox,
    QVBoxLayout, QWidget,
)


# Unit wheel speed contribution (left, right) of each direction key
DIRECTIONS = {
    'forward': (1.0, 1.0),
    'right': (0.5, -0.5),
    'backward': (-1.0, -1.0),
    'left': (-0.5, 0.5),
}

KEY_TO_DIRECTION = {
    Qt.Key_Up: 'forward', Qt.Key_W: 'forward',
    Qt.Key_Right: 'right', Qt.Key_D: 'right',
    Qt.Key_Down: 'backward', Qt.Key_S: 'backward',
    Qt.Key_Left: 'left', Qt.Key_A: 'left',
}

TOGGLE_MODE_KEY = Qt.Key_X
PRINT_POSE_KEY = Qt.Key_Z

CONTROL_RATE = 20.0
SPIN_PER_STEP = 5


def directions_to_wheels(directions):
    """Sum the unit contributions of held directions; scale down keeping the ratio if above 1."""
    left = sum(DIRECTIONS[d][0] for d in directions)
    right = sum(DIRECTIONS[d][1] for d in directions)
    peak = max(abs(left), abs(right))
    if peak > 1.0:
        left, right = left / peak, right / peak
    return left, right


def ramp_towards(current, target, max_step):
    """Step (left, right) towards target; larger component limited to max_step, ratio kept."""
    d_left, d_right = target[0] - current[0], target[1] - current[1]
    peak = max(abs(d_left), abs(d_right))
    if peak <= max_step:
        return target
    scale = max_step / peak
    return current[0] + d_left * scale, current[1] + d_right * scale


class ControlPanelNode(Node):

    def __init__(self):
        super().__init__('control_panel')

        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('wheel_separation', 0.32)
        self.declare_parameter('default_rpm', 80)
        self.declare_parameter('max_rpm', 150)
        self.declare_parameter('ramp_rate', 0.5)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('fallback_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_footprint')

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter(
            'wheel_separation').get_parameter_value().double_value
        self.default_rpm = self.get_parameter('default_rpm').get_parameter_value().integer_value
        self.max_rpm = self.get_parameter('max_rpm').get_parameter_value().integer_value
        # Wheel rim acceleration limit [m/s^2]
        self.ramp_rate = self.get_parameter('ramp_rate').get_parameter_value().double_value
        self.pose_frames = [
            self.get_parameter('target_frame').get_parameter_value().string_value,
            self.get_parameter('fallback_frame').get_parameter_value().string_value,
        ]
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value

        self.mode = '-'
        self.last_pose_text = '-'
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)
        self.toggle_client = self.create_client(Trigger, '/mode_manager/toggle_mode')

        # Own listener node on its own thread: /tf is too busy for the panel's spin_once
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, None, spin_thread=True)

        mode_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(String, 'robot_mode', self.mode_cb, mode_qos)
        self.nav_status = '-'
        self.create_subscription(String, 'nav_status', self.nav_status_cb, 10)
        # Command actually sent to the robot; shown in AUTO, where the panel's own command is muted
        self.cmd_vel = Twist()
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 1)

    def mode_cb(self, msg):
        self.mode = msg.data

    def cmd_vel_cb(self, msg):
        self.cmd_vel = msg

    def nav_status_cb(self, msg):
        self.nav_status = msg.data
        print(f'[NAV] {msg.data}', flush=True)

    def toggle_mode(self):
        if not self.toggle_client.service_is_ready():
            print('[MODE] /mode_manager/toggle_mode service not available '
                  '(is mode_manager running?)', flush=True)
            return
        future = self.toggle_client.call_async(Trigger.Request())
        future.add_done_callback(self.toggle_done_cb)

    def toggle_done_cb(self, future):
        result = future.result()
        if result is None or not result.success:
            print('[MODE] mode change failed', flush=True)
        else:
            print(f'[MODE] {result.message}', flush=True)

    def print_pose(self):
        for frame in self.pose_frames:
            try:
                tf = self.tf_buffer.lookup_transform(frame, self.robot_frame, Time())
            except TransformException:
                continue
            t = tf.transform.translation
            q = tf.transform.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.last_pose_text = (
                f'x={t.x:.2f} m, y={t.y:.2f} m, yaw={math.degrees(yaw):.1f}°')
            if frame != self.pose_frames[0]:
                self.last_pose_text += f' ({frame})'
            print(f'[POSE] {self.last_pose_text}', flush=True)
            return
        self.last_pose_text = (
            f'No TF found ({" / ".join(self.pose_frames)} → {self.robot_frame})')
        print(f'[POSE] {self.last_pose_text}', flush=True)

    def rpm_to_speed(self, rpm):
        return rpm * 2.0 * math.pi / 60.0 * self.wheel_radius

    def speed_to_rpm(self, speed):
        return speed / self.wheel_radius * 60.0 / (2.0 * math.pi)

    def publish_wheels(self, left, right):
        msg = Twist()
        msg.linear.x = (left + right) / 2.0
        msg.angular.z = (right - left) / self.wheel_separation
        self.cmd_pub.publish(msg)
        return msg

    def twist_to_wheels(self, twist):
        half_turn = twist.angular.z * self.wheel_separation / 2.0
        return twist.linear.x - half_turn, twist.linear.x + half_turn


class ControlPanelWindow(QWidget):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.held = set()
        self.held_commands = set()
        self.wheels = (0.0, 0.0)

        self.setWindowTitle('Robot Control Panel')
        self.build_ui()
        self.resize(380,550)

        QApplication.instance().installEventFilter(self)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.control_step)
        self.timer.start(int(1000 / CONTROL_RATE))

    def build_ui(self):
        keys_box = QGroupBox('Drive')
        keys_grid = QGridLayout(keys_box)
        self.key_labels = {}
        for direction, text, row, col in (
            ('forward', '↑ / W\nForward', 0, 1),
            ('left', '← / A\nACW', 1, 0),
            ('backward', '↓ / S\nBackward', 1, 1),
            ('right', '→ / D\nCW', 1, 2),
        ):
            label = QLabel(text)
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumSize(80, 50)
            self.key_labels[direction] = label
            keys_grid.addWidget(label, row, col)

        self.refresh_key_labels()

        command_box = QGroupBox('Commands')
        command_row = QHBoxLayout(command_box)
        self.command_labels = {}
        for key, text in ((TOGGLE_MODE_KEY, 'X: Toggle Mode'), (PRINT_POSE_KEY, 'Z: Print Pose')):
            label = QLabel(text)
            label.setAlignment(Qt.AlignCenter)
            self.command_labels[key] = label
            command_row.addWidget(label)

        self.refresh_command_labels()

        speed_box = QGroupBox('Maximum RPM')
        speed_row = QHBoxLayout(speed_box)
        self.rpm_slider = QSlider(Qt.Horizontal)
        self.rpm_slider.setRange(0, self.node.max_rpm)
        self.rpm_slider.setValue(self.node.default_rpm)
        self.rpm_slider.setFocusPolicy(Qt.NoFocus)
        self.rpm_spin = QSpinBox()
        self.rpm_spin.setRange(0, self.node.max_rpm)
        self.rpm_spin.setValue(self.node.default_rpm)
        self.rpm_spin.setSuffix(' RPM')
        self.rpm_slider.valueChanged.connect(self.rpm_spin.setValue)
        self.rpm_spin.valueChanged.connect(self.rpm_slider.setValue)
        self.rpm_limit_label = QLabel()
        self.rpm_spin.valueChanged.connect(self.refresh_rpm_limit_label)
        self.refresh_rpm_limit_label()
        speed_row.addWidget(self.rpm_slider, 1)
        speed_row.addWidget(self.rpm_spin)
        speed_row.addWidget(self.rpm_limit_label)

        status_box = QGroupBox('State')
        status_layout = QVBoxLayout(status_box)
        self.mode_label = QLabel()
        self.wheel_label = QLabel()
        self.twist_label = QLabel()
        self.pose_label = QLabel()
        self.nav_label = QLabel()
        for label in (self.mode_label, self.wheel_label, self.twist_label, self.pose_label,
                      self.nav_label):
            status_layout.addWidget(label)

        layout = QVBoxLayout(self)
        layout.addWidget(keys_box)
        layout.addWidget(command_box)
        layout.addWidget(speed_box)
        layout.addWidget(status_box)

    def refresh_key_labels(self):
        for direction, label in self.key_labels.items():
            active = direction in self.held
            label.setStyleSheet(
                'border: 1px solid gray; border-radius: 4px;'
                + ('background: #2e7d32; color: white; font-weight: bold;' if active else ''))

    def refresh_command_labels(self):
        for key, label in self.command_labels.items():
            active = key in self.held_commands
            label.setStyleSheet(
                'border: 1px solid gray; border-radius: 4px; padding: 4px;'
                + ('background: #2e7d32; color: white; font-weight: bold;' if active else ''))

    def refresh_rpm_limit_label(self):
        self.rpm_limit_label.setText(
            f'≈ {self.node.rpm_to_speed(self.rpm_spin.value()):.2f} m/s')

    def eventFilter(self, obj, event):
        if event.type() in (QEvent.KeyPress, QEvent.KeyRelease):
            if event.key() in (TOGGLE_MODE_KEY, PRINT_POSE_KEY):
                # One action per real press; ignore releases and auto-repeat
                if not event.isAutoRepeat():
                    if event.type() == QEvent.KeyPress:
                        self.held_commands.add(event.key())
                        if event.key() == TOGGLE_MODE_KEY:
                            self.node.toggle_mode()
                        else:
                            self.node.print_pose()
                    else:
                        self.held_commands.discard(event.key())
                    self.refresh_command_labels()
                return True
            direction = KEY_TO_DIRECTION.get(event.key())
            if direction is None:
                return False

            if not event.isAutoRepeat():
                if event.type() == QEvent.KeyPress:
                    self.held.add(direction)
                else:
                    self.held.discard(direction)
                self.refresh_key_labels()
            return True
        if event.type() == QEvent.WindowDeactivate and obj is self:

            self.held.clear()
            self.held_commands.clear()
            self.refresh_key_labels()
            self.refresh_command_labels()
        return False

    def control_step(self):
        max_speed = self.node.rpm_to_speed(self.rpm_spin.value())
        unit_left, unit_right = directions_to_wheels(self.held)
        target = (unit_left * max_speed, unit_right * max_speed)
        self.wheels = ramp_towards(self.wheels, target, self.node.ramp_rate / CONTROL_RATE)

        msg = self.node.publish_wheels(*self.wheels)
        wheels = self.wheels

        mode = self.node.mode
        if mode == 'AUTO':
            msg = self.node.cmd_vel
            wheels = self.node.twist_to_wheels(msg)
        self.mode_label.setText(f'Mode: {mode}')
        self.wheel_label.setText(
            f'Wheel: Left {self.node.speed_to_rpm(wheels[0]):+6.1f} RPM, '
            f' Right {self.node.speed_to_rpm(wheels[1]):+6.1f} RPM')
        self.twist_label.setText(
            f'Speed: v = {msg.linear.x:+.2f} m/s, ω = {msg.angular.z:+.2f} rad/s')
        self.pose_label.setText(f'Last Pose: {self.node.last_pose_text}')
        self.nav_label.setText(f'Navigation: {self.node.nav_status}')

        # spin_once runs a single callback; /cmd_vel alone arrives at the loop rate
        for _ in range(SPIN_PER_STEP):
            rclpy.spin_once(self.node, timeout_sec=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ControlPanelNode()
    app = QApplication(sys.argv)
    window = ControlPanelWindow(node)
    window.show()


    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())

    try:
        app.exec_()
    finally:
        node.publish_wheels(0.0, 0.0)

        node.tf_listener.executor.shutdown()
        node.tf_listener.dedicated_listener_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
