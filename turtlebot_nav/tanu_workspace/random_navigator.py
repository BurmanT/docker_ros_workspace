#!/usr/bin/env python3
"""
random_navigator.py

Sends TurtleBot to hardcoded goal positions around the table.
Run this on the TurtleBot laptop.

Usage:
  python3 ~/random_navigator.py

Services:
  /next_goal  — sends TurtleBot to next position in sequence
  /random_goal — sends TurtleBot to a random position
"""
import random
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


# Goal positions around the table in map frame
# (x, y, orientation_z, orientation_w)
TABLE_POSITIONS = [
    (-0.5149,  0.1722, -0.0502,  0.9987),   # position 1
    (-0.5826, -0.5846,  0.4207,  0.9072),   # position 2
]


class RandomNavigator(Node):
    def __init__(self):
        super().__init__("random_navigator")

        self._client = ActionClient(
            self, NavigateToPose, "navigate_to_pose"
        )

        self._current_index = 0

        self.create_service(
            Trigger, "next_goal", self._next_goal_callback
        )
        self.create_service(
            Trigger, "random_goal", self._random_goal_callback
        )

        self.get_logger().info("Random navigator ready.")
        self.get_logger().info(f"{len(TABLE_POSITIONS)} goal positions loaded.")
        self.get_logger().info("Call /next_goal to go to next position.")
        self.get_logger().info("Call /random_goal to go to random position.")

    # ------------------------------------------------------------------
    def _next_goal_callback(self, request, response):
        x, y, qz, qw = TABLE_POSITIONS[self._current_index]
        self.get_logger().info(
            f"Going to position {self._current_index + 1}: ({x}, {y})"
        )
        self._send_goal(x, y, qz, qw)

        # Advance to next position (wraps around)
        self._current_index = (self._current_index + 1) % len(TABLE_POSITIONS)

        response.success = True
        response.message = f"Navigating to ({x:.2f}, {y:.2f})"
        return response

    # ------------------------------------------------------------------
    def _random_goal_callback(self, request, response):
        x, y, qz, qw = random.choice(TABLE_POSITIONS)
        self.get_logger().info(f"Going to random position: ({x}, {y})")
        self._send_goal(x, y, qz, qw)

        response.success = True
        response.message = f"Navigating to ({x:.2f}, {y:.2f})"
        return response

    # ------------------------------------------------------------------
    def _send_goal(self, x, y, qz, qw):
        self._client.wait_for_server()

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id       = "map"
        goal.pose.header.stamp          = self.get_clock().now().to_msg()
        goal.pose.pose.position.x       = x
        goal.pose.pose.position.y       = y
        goal.pose.pose.orientation.z    = qz
        goal.pose.pose.orientation.w    = qw

        future = self._client.send_goal_async(
            goal,
            feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)

    # ------------------------------------------------------------------
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return
        self.get_logger().info("Goal accepted — TurtleBot navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        self.get_logger().info("TurtleBot arrived at goal!")
        self.get_logger().info(
            "Now trigger /update_kinova_goal to move Kinova arm."
        )

    def _feedback_callback(self, feedback):
        pass


# ------------------------------------------------------------------
def main():
    rclpy.init()
    node = RandomNavigator()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()