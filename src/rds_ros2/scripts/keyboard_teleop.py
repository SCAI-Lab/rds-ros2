#!/usr/bin/env python3

import sys
import tty
import termios
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

LINEAR_STEP  = 0.1   # m/s per keypress
ANGULAR_STEP = 0.1   # rad/s per keypress
LINEAR_MAX   = 1.5
ANGULAR_MAX  = 2.0

HELP = """
RDS Keyboard Teleop
-------------------
  W / S  : forward / backward
  A / D  : turn left / right
  Space  : stop
  Q      : quit
"""


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('topic', '/cmd_vel_in')
        topic = self.get_parameter('topic').value
        self.pub = self.create_publisher(Twist, topic, 10)
        self.linear  = 0.0
        self.angular = 0.0
        self.get_logger().info(f'Publishing to {topic}')

    def publish(self):
        msg = Twist()
        msg.linear.x  = self.linear
        msg.angular.z = self.angular
        self.pub.publish(msg)

    def clamp(self, val, limit):
        return max(-limit, min(limit, val))


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()

    settings = termios.tcgetattr(sys.stdin)
    print(HELP)

    try:
        while rclpy.ok():
            key = get_key(settings)

            if key == 'w':
                node.linear  = node.clamp(node.linear  + LINEAR_STEP,  LINEAR_MAX)
            elif key == 's':
                node.linear  = node.clamp(node.linear  - LINEAR_STEP,  LINEAR_MAX)
            elif key == 'a':
                node.angular = node.clamp(node.angular + ANGULAR_STEP, ANGULAR_MAX)
            elif key == 'd':
                node.angular = node.clamp(node.angular - ANGULAR_STEP, ANGULAR_MAX)
            elif key == ' ':
                node.linear  = 0.0
                node.angular = 0.0
            elif key in ('q', 'Q', '\x03'):   # q or Ctrl-C
                break

            print(f'\rlinear: {node.linear:+.2f} m/s   angular: {node.angular:+.2f} rad/s   ', end='')
            node.publish()

    finally:
        # Send stop on exit
        stop = Twist()
        node.pub.publish(stop)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
