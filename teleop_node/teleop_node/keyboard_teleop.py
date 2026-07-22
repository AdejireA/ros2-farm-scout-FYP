"""
Keyboard teleoperation node.

Publishes geometry_msgs/Twist to /cmd_vel_teleop at 10 Hz.

Controls (must run in a real terminal — not rqt/IDE output pane):
  w / UP    : forward
  s / DOWN  : backward
  a / LEFT  : turn left (counter-clockwise)
  d / RIGHT : turn right (clockwise)
  q         : forward + left
  e         : forward + right
  z         : backward + left
  c         : backward + right
  SPACE / x : full stop
  k         : quit
"""

import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

# ── Velocity limits ─────────────────────────────────────────────────────────
MAX_LIN = 0.30   # m/s
MAX_ANG = 0.80   # rad/s

# ── Key → (linear_x, angular_z) mapping ─────────────────────────────────────
KEY_MAP = {
    'w':        ( MAX_LIN,  0.0),
    's':        (-MAX_LIN,  0.0),
    'a':        ( 0.0,      MAX_ANG),
    'd':        ( 0.0,     -MAX_ANG),
    'q':        ( MAX_LIN,  MAX_ANG),
    'e':        ( MAX_LIN, -MAX_ANG),
    'z':        (-MAX_LIN,  MAX_ANG),
    'c':        (-MAX_LIN, -MAX_ANG),
    ' ':        ( 0.0,      0.0),
    'x':        ( 0.0,      0.0),
    # arrow keys arrive as 3-byte escape sequences → handled below
    '\x1b[A':   ( MAX_LIN,  0.0),   # UP
    '\x1b[B':   (-MAX_LIN,  0.0),   # DOWN
    '\x1b[C':   ( 0.0,     -MAX_ANG),  # RIGHT
    '\x1b[D':   ( 0.0,      MAX_ANG),  # LEFT
}

QUIT_KEYS = {'k', '\x03'}   # k or Ctrl-C

BANNER = """
──────────────────────────────────────────────
  Agricultural Robot  |  TELEOP MODE
──────────────────────────────────────────────
  w/↑  forward      s/↓  backward
  a/←  turn left    d/→  turn right
  q    fwd+left     e    fwd+right
  z    bwd+left     c    bwd+right
  SPACE / x  STOP
  k          quit
──────────────────────────────────────────────
  Publishing → /cmd_vel_teleop
"""

# Time without a keypress before the robot is commanded to stop (seconds)
KEY_TIMEOUT = 0.40


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        # TwistStamped required: arbitration_node and Gazebo Harmonic bridge expect stamped velocity
        self._pub = self.create_publisher(TwistStamped, '/cmd_vel_teleop', 10)
        self._timer = self.create_timer(0.10, self._publish_cb)   # 10 Hz

        self._lin: float = 0.0
        self._ang: float = 0.0
        self._last_key_time: float = self.get_clock().now().nanoseconds * 1e-9
        self._lock = threading.Lock()
        self._running = True

        self._kbd_thread = threading.Thread(target=self._read_keys, daemon=True)
        self._kbd_thread.start()

    # ── Timer callback: publish current velocities ───────────────────────────
    def _publish_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            elapsed = now - self._last_key_time
            if elapsed > KEY_TIMEOUT:
                self._lin = 0.0
                self._ang = 0.0
            lin, ang = self._lin, self._ang

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = lin
        msg.twist.angular.z = ang
        self._pub.publish(msg)

    # ── Key-reading thread ────────────────────────────────────────────────────
    def _read_keys(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self._running:
                # Non-blocking check with 0.1 s timeout
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not rlist:
                    continue

                ch = sys.stdin.read(1)

                # Arrow keys: ESC [ A/B/C/D
                if ch == '\x1b':
                    more, _, _ = select.select([sys.stdin], [], [], 0.05)
                    if more:
                        ch += sys.stdin.read(2)

                if ch in QUIT_KEYS:
                    self._running = False
                    rclpy.shutdown()
                    break

                if ch in KEY_MAP:
                    lin, ang = KEY_MAP[ch]
                    with self._lock:
                        self._lin = lin
                        self._ang = ang
                        self._last_key_time = (
                            self.get_clock().now().nanoseconds * 1e-9
                        )
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    print(BANNER)

    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Send a stop command before exiting
        stop = TwistStamped()
        stop.header.stamp = node.get_clock().now().to_msg()
        stop.header.frame_id = 'base_link'
        node._pub.publish(stop)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
