#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")
        self._pub = self.create_publisher(Twist, "/commands/velocity", 10)
        self.create_subscription(Twist, "/cmd_vel", self._callback, 10)
        self.get_logger().info("Relaying /cmd_vel → /commands/velocity")

    def _callback(self, msg):
        self._pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(CmdVelRelay())
    rclpy.shutdown()

if __name__ == "__main__":
    main()