#!/usr/bin/env python3
"""
test_publisher.py
Run on the TurtleBot laptop to simulate arriving at a goal.
Usage: python3 test_publisher.py
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class TestPublisher(Node):
    def __init__(self):
        super().__init__("test_publisher")
        self._pub = self.create_publisher(Int32, "/turtlebot_goal_index", 10)
        # Publish once per second
        self.create_timer(1.0, self._publish)
        self.get_logger().info("Publishing to /turtlebot_goal_index every second...")

    def _publish(self):
        msg = Int32()
        msg.data = 0   # simulate arriving at goal 0
        self._pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")


def main():
    rclpy.init()
    node = TestPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()