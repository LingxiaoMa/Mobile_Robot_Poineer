import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# ── Button indices (PS4: 0=X, 1=O, 2=Square) ──
BUTTON_START_WP = 0   # X      → start GPS waypoint mission (needs L1+L2)
BUTTON_MANUAL   = 1   # O      → cancel mission + manual mode
BUTTON_ESTOP    = 2   # Square → emergency stop
BUTTON_L1       = 9   # L1     } safety lock:
BUTTON_L2       = 10  # L2     } both must be held to move

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

        self._cmd_pub     = self.create_publisher(Twist,  '/cmd_vel',        10)
        self._mission_pub = self.create_publisher(String, '/mission_control', 10)

        self.create_subscription(Joy,   '/joy',          self._joy_cb,  10)
        self.create_subscription(Twist, '/cmd_vel_auto', self._auto_cb, 10)

        self.get_logger().info(
            'MasterController ready — STANDBY\n'
            f'  X({BUTTON_START_WP}) → START GPS mission\n'
            f'  O({BUTTON_MANUAL})   → CANCEL mission + MANUAL\n'
            f'  Square({BUTTON_ESTOP}) → ESTOP')

    def _joy_cb(self, msg: Joy):
        buttons = list(msg.buttons)

        while len(self._prev_buttons) < len(buttons):
            self._prev_buttons.append(0)

        def just_pressed(idx):
            return (idx < len(buttons)
                    and buttons[idx] == 1
                    and self._prev_buttons[idx] == 0)

        # ── button events ──────────────────────────────────────────────
        if just_pressed(BUTTON_ESTOP):
            self._publish_stop()
            self._publish_mission('CANCEL')
            self._state = self.STANDBY
            self.get_logger().info('ESTOP — stopped, STANDBY')

        elif just_pressed(BUTTON_MANUAL):
            self._publish_mission('CANCEL')
            self._state = self.MANUAL
            self.get_logger().info('O → CANCEL mission + MANUAL')

        elif just_pressed(BUTTON_START_WP):
            self._publish_stop()
            self._publish_mission('START')
            self._state = self.AUTO
            self.get_logger().info('X → START GPS mission')

        # ── MANUAL: publish joystick axes ──────────────────────────────
        if self._state == self.MANUAL:
            twist = Twist()
            if len(msg.axes) > max(LINEAR_AXIS, ANGULAR_AXIS):
                twist.linear.x  = msg.axes[LINEAR_AXIS]  * LINEAR_SCALE
                twist.angular.z = msg.axes[ANGULAR_AXIS] * ANGULAR_SCALE
            self._cmd_pub.publish(twist)

        self._prev_buttons = buttons

    def _auto_cb(self, msg: Twist):
        if self._state == self.AUTO:
            self._cmd_pub.publish(msg)

    def _publish_stop(self):
        self._cmd_pub.publish(Twist())

    def _publish_mission(self, cmd: str):
        msg = String()
        msg.data = cmd
        self._mission_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MasterController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
