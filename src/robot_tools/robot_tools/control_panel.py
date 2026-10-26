import math
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

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

CONTROL_RATE = 20.0


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

        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter(
            'wheel_separation').get_parameter_value().double_value
        self.default_rpm = self.get_parameter('default_rpm').get_parameter_value().integer_value
        self.max_rpm = self.get_parameter('max_rpm').get_parameter_value().integer_value
        # Wheel rim acceleration limit [m/s^2]
        self.ramp_rate = self.get_parameter('ramp_rate').get_parameter_value().double_value

        self.mode = '?'
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_teleop', 10)

        mode_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(String, 'robot_mode', self.mode_cb, mode_qos)

    def mode_cb(self, msg):
        self.mode = msg.data

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


class ControlPanelWindow(QWidget):

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.held = set()
        self.wheels = (0.0, 0.0)

        self.setWindowTitle('Robot Control Panel')
        self.build_ui()

        # Drive keys are caught application-wide so the spin box / slider never swallow them
        QApplication.instance().installEventFilter(self)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.control_step)
        self.timer.start(int(1000 / CONTROL_RATE))

    def build_ui(self):
        keys_box = QGroupBox('Sürüş (pencere odaktayken basılı tut)')
        keys_grid = QGridLayout(keys_box)
        self.key_labels = {}
        for direction, text, row, col in (
            ('forward', '↑ / W\nİleri', 0, 1),
            ('left', '← / A\nSol', 1, 0),
            ('backward', '↓ / S\nGeri', 1, 1),
            ('right', '→ / D\nSağ', 1, 2),
        ):
            label = QLabel(text)
            label.setAlignment(Qt.AlignCenter)
            label.setMinimumSize(80, 50)
            self.key_labels[direction] = label
            keys_grid.addWidget(label, row, col)
        hint = QLabel('Tuşlar birleştirilebilir (örn. ↑+→ sağa viraj).\n'
                      'Tuş bırakılınca robot yumuşakça durur.')
        hint.setAlignment(Qt.AlignCenter)
        keys_grid.addWidget(hint, 2, 0, 1, 3)
        self.refresh_key_labels()

        speed_box = QGroupBox('Maksimum teker hızı (RPM)')
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

        status_box = QGroupBox('Durum')
        status_layout = QVBoxLayout(status_box)
        self.mode_label = QLabel()
        self.wheel_label = QLabel()
        self.twist_label = QLabel()
        for label in (self.mode_label, self.wheel_label, self.twist_label):
            status_layout.addWidget(label)

        layout = QVBoxLayout(self)
        layout.addWidget(keys_box)
        layout.addWidget(speed_box)
        layout.addWidget(status_box)

    def refresh_key_labels(self):
        for direction, label in self.key_labels.items():
            active = direction in self.held
            label.setStyleSheet(
                'border: 1px solid gray; border-radius: 4px;'
                + ('background: #2e7d32; color: white; font-weight: bold;' if active else ''))

    def refresh_rpm_limit_label(self):
        self.rpm_limit_label.setText(
            f'≈ {self.node.rpm_to_speed(self.rpm_spin.value()):.2f} m/s')

    def eventFilter(self, obj, event):
        if event.type() in (QEvent.KeyPress, QEvent.KeyRelease):
            direction = KEY_TO_DIRECTION.get(event.key())
            if direction is None:
                return False
            # Holding a key makes X11 emit repeated press/release pairs; only real ones count
            if not event.isAutoRepeat():
                if event.type() == QEvent.KeyPress:
                    self.held.add(direction)
                else:
                    self.held.discard(direction)
                self.refresh_key_labels()
            return True
        if event.type() == QEvent.WindowDeactivate and obj is self:
            # Releases are not delivered once focus is gone; don't keep driving
            self.held.clear()
            self.refresh_key_labels()
        return False

    def control_step(self):
        max_speed = self.node.rpm_to_speed(self.rpm_spin.value())
        unit_left, unit_right = directions_to_wheels(self.held)
        target = (unit_left * max_speed, unit_right * max_speed)
        self.wheels = ramp_towards(self.wheels, target, self.node.ramp_rate / CONTROL_RATE)

        msg = self.node.publish_wheels(*self.wheels)

        mode = self.node.mode
        note = '' if mode == 'MANUAL' else '  (panel komutları yok sayılıyor)'
        self.mode_label.setText(f'Mod: {mode}{note}')
        self.wheel_label.setText(
            f'Teker: sol {self.node.speed_to_rpm(self.wheels[0]):+6.1f} RPM, '
            f'sağ {self.node.speed_to_rpm(self.wheels[1]):+6.1f} RPM')
        self.twist_label.setText(
            f'cmd_vel_teleop: v = {msg.linear.x:+.2f} m/s, ω = {msg.angular.z:+.2f} rad/s')

        rclpy.spin_once(self.node, timeout_sec=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = ControlPanelNode()
    app = QApplication(sys.argv)
    window = ControlPanelWindow(node)
    window.show()

    # Qt's event loop blocks Python signal handling; quit cleanly on Ctrl+C / launch shutdown
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())

    try:
        app.exec_()
    finally:
        node.publish_wheels(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
