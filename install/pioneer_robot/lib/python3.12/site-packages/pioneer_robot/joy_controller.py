import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

# ── Button indices (adjust if your controller mapping differs) ──
BUTTON_AUTO   = 0   # → autonomous mode
BUTTON_MANUAL = 1   # → manual mode
BUTTON_ESTOP  = 2   # → emergency stop + standby

# ── Joystick axes for manual drive ──
LINEAR_AXIS   = 1   # left stick Y
ANGULAR_AXIS  = 0   # left stick X
LINEAR_SCALE  = 0.5
ANGULAR_SCALE = 1.0


class MasterController(Node):

    STANDBY = 'STANDBY'
    MANUAL  = 'MANUAL'
    AUTO    = 'AUTO'

    def __init__(self):
        super().__init__('joy_controller')
        self._state        = self.STANDBY
        self._prev_buttons = []

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Joy,   '/joy',          self._joy_cb,  10)
        self.create_subscription(Twist, '/cmd_vel_auto', self._auto_cb, 10)

        self.get_logger().info(
            'MasterController ready — STANDBY\n'
            f'  Button {BUTTON_MANUAL} → MANUAL\n'
            f'  Button {BUTTON_AUTO}   → AUTO\n'
            f'  Button {BUTTON_ESTOP}  → ESTOP')

    # ── Joy callback ────────────────────────────────────────────────
    def _joy_cb(self, msg: Joy):
        buttons = list(msg.buttons)

        while len(self._prev_buttons) < len(buttons):
            self._prev_buttons.append(0)

        # Log any button state change for debugging
        if buttons != self._prev_buttons[:len(buttons)]:
            self.get_logger().info(
                f'[DEBUG] buttons={buttons}  prev={self._prev_buttons[:len(buttons)]}  state={self._state}')

        def just_pressed(idx):
            return (idx < len(buttons)
                    and buttons[idx] == 1
                    and self._prev_buttons[idx] == 0)

        # Emergency stop — highest priority, works from any state
        if just_pressed(BUTTON_ESTOP):
            self._publish_stop()
            self._state = self.STANDBY
            self.get_logger().info('ESTOP — robot stopped, back to STANDBY')

        elif just_pressed(BUTTON_MANUAL) and self._state != self.MANUAL:
            self._state = self.MANUAL
            self.get_logger().info('→ MANUAL mode')

        elif just_pressed(BUTTON_AUTO) and self._state != self.AUTO:
            self._publish_stop()   # stop before handing over to local_controller
            self._state = self.AUTO
            self.get_logger().info('→ AUTO mode')

        # In MANUAL: publish cmd_vel from joystick axes
        if self._state == self.MANUAL:
            twist = Twist()
            if len(msg.axes) > max(LINEAR_AXIS, ANGULAR_AXIS):
                twist.linear.x  = msg.axes[LINEAR_AXIS]  * LINEAR_SCALE
                twist.angular.z = msg.axes[ANGULAR_AXIS] * ANGULAR_SCALE
            self._cmd_pub.publish(twist)

        self._prev_buttons = buttons

    # ── Auto relay ──────────────────────────────────────────────────
    def _auto_cb(self, msg: Twist):
        if self._state == self.AUTO:
            self._cmd_pub.publish(msg)

    def _publish_stop(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MasterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
