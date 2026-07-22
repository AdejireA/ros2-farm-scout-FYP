"""
Mode-switching arbitration node.

Subscriptions:
  /mode          (std_msgs/String)   — "auto" | "manual"
  /cmd_vel_auto  (geometry_msgs/Twist) — velocity from Nav2 (remapped from /cmd_vel)
  /cmd_vel_teleop(geometry_msgs/Twist) — velocity from keyboard teleop

Publisher:
  /cmd_vel       (geometry_msgs/Twist) — forwarded to robot / Gazebo bridge

The node defaults to MANUAL mode for safety.  Publishing "auto" to /mode
hands control to Nav2.  If no auto command arrives within AUTO_TIMEOUT seconds
the robot is stopped and a warning is logged.

Switch modes at runtime:
  ros2 topic pub /mode std_msgs/msg/String "data: 'auto'"
  ros2 topic pub /mode std_msgs/msg/String "data: 'manual'"
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String

AUTO_TIMEOUT = 0.5   # seconds — stop robot if Nav2 goes silent


class ArbMode:
    MANUAL = 'manual'
    AUTO   = 'auto'


class ArbitrationNode(Node):

    def __init__(self):
        super().__init__('arbitration_node')

        # ── State ─────────────────────────────────────────────────────────────
        self._mode: str = ArbMode.MANUAL
        self._auto_cmd   = TwistStamped()
        self._teleop_cmd = TwistStamped()
        self._last_auto_time: float = 0.0   # epoch seconds

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(String,       '/mode',           self._mode_cb,   10)
        # TwistStamped required: Gazebo Harmonic diff-drive bridge expects stamped velocity
        self.create_subscription(TwistStamped, '/cmd_vel_auto',   self._auto_cb,   10)
        self.create_subscription(TwistStamped, '/cmd_vel_teleop', self._teleop_cb, 10)

        # ── Publisher ─────────────────────────────────────────────────────────
        # TwistStamped required: ros_gz_bridge maps TwistStamped→gz.msgs.Twist for Gazebo Harmonic
        self._cmd_pub    = self.create_publisher(TwistStamped, '/cmd_vel',      10)
        self._status_pub = self.create_publisher(String,       '/current_mode', 10)

        # ── Output timer (20 Hz) ──────────────────────────────────────────────
        self.create_timer(0.05, self._forward_cb)

        # ── Status log timer (1 Hz) ───────────────────────────────────────────
        self.create_timer(1.0,  self._status_cb)

        self.get_logger().info(
            'ArbitrationNode ready — default mode: MANUAL\n'
            '  Switch: ros2 topic pub /mode std_msgs/msg/String "data: \'auto\'"'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _stamped_stop(self) -> TwistStamped:
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = 'base_link'
        return stop

    def _mode_cb(self, msg: String):
        requested = msg.data.strip().lower()
        if requested not in (ArbMode.MANUAL, ArbMode.AUTO):
            self.get_logger().warn(
                f"Unknown mode '{msg.data}'. Valid values: 'auto', 'manual'."
            )
            return
        if requested != self._mode:
            self.get_logger().info(
                f'Mode switched: {self._mode.upper()} → {requested.upper()}'
            )
            self._mode = requested
            if requested == ArbMode.MANUAL:
                # Immediately stop robot when returning to manual
                self._cmd_pub.publish(self._stamped_stop())

    def _auto_cb(self, msg: TwistStamped):
        self._auto_cmd = msg
        self._last_auto_time = self.get_clock().now().nanoseconds * 1e-9

    def _teleop_cb(self, msg: TwistStamped):
        self._teleop_cmd = msg

    # ── Main forwarding loop ───────────────────────────────────────────────────

    def _forward_cb(self):
        if self._mode == ArbMode.AUTO:
            now = self.get_clock().now().nanoseconds * 1e-9
            if (self._last_auto_time > 0.0 and
                    now - self._last_auto_time > AUTO_TIMEOUT):
                self.get_logger().warn(
                    'No /cmd_vel_auto received for '
                    f'{AUTO_TIMEOUT} s — stopping robot.',
                    throttle_duration_sec=2.0,
                )
                self._cmd_pub.publish(self._stamped_stop())
                return
            self._cmd_pub.publish(self._auto_cmd)
        else:
            self._cmd_pub.publish(self._teleop_cmd)

    def _status_cb(self):
        msg = String()
        msg.data = self._mode
        self._status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArbitrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
